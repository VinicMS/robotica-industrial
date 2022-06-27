"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Industrial - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem 
   necessidade de clicar nos botões do CoppeliaSim
"""

import vrep
import time
import sys
import numpy as np

vrep.simxFinish(-1) # just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
   print ('Servidor conectado!') 
else:
    print ('Problemas para conectar o servidor!')
    sys.exit()

#Ativa modo síncrono da RemoteAPI
vrep.simxSynchronous(clientID, True) 

#Inicia a simulação
vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);

# Juntas do Robô
_, J0Handle = vrep.simxGetObjectHandle(clientID,'J0',vrep.simx_opmode_oneshot_wait)
_, J1Handle = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_oneshot_wait)
_, J2Handle = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_oneshot_wait)
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)
# Alvo
_, AlvoHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)


#Função que obtém a pose de um objeto da cena
#Simula algoritmos de localização global [Ex. Visão Computacional]
def Obter_Pose(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_streaming)
    _, ori = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_streaming)

    x = pos[0]   
    y = pos[1]
    z = pos[2]
    phi = ori[0]
    theta = ori[1]
    psi = ori[2]
    
    return x, y, z, phi, theta, psi

#Função que obtem o ângulo de uma junta
#Simula encoders absolutos nas juntas
def Obter_Angulo_Junta(handle): 
    _, q_medido = vrep.simxGetJointPosition(clientID,handle,vrep.simx_opmode_streaming)
    return q_medido

# Envia referência de ângulo para juntas
def Setar_Juntas_Robo(q0, q1, q2): 
   #Aplica cálculos ao objeto no simulador
   vrep.simxSetJointTargetPosition(clientID, J0Handle, q0, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J1Handle, q1, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J2Handle, q2, vrep.simx_opmode_streaming)


######################################################################################
############################ CINEMÁTICA INVERSA ######################################
######################################################################################
def cinematica_inv_num_Ji(xd, yd, zd, q0_m, q1_m, q2_m, dt):
    # Inicialização do vetor q
    # Atenção: Esta posição inicial gera uma matriz singular (det(J)=0)
    a0 = 0.3
    a1 = 0.5
    a2 = 0.5
    
    q = np.array(( (q0_m),  
                   (q1_m),
                   (q2_m)
                ))
    
    r = np.sqrt(xd**2+yd**2+zd**2)
    
    if r >= a0-a1-a2 and r <= a0+a1+a2: # Workspace
        #Inicializa vetor Cinemática Direta
        Cd = np.array(( ( ) ),  
                        ( ) ),
                        ( ) )
                     ))

        E = np.array (( ( xd - Cd[0] ), #Inicializa vetor erro de posição
                        ( yd - Cd[1] ),
                        ( zd - Cd[2] ),
                        ( 0 ), #Ignora erro de rotação
                        ( 0 ),
                        ( 0 )
                    ))
        
        # ReCalcula/Atualiza o Jacobiano em função de q
        J = np.array(( (   ), 
                       (   ),
                       (   ),
                       (   ),
                       (   ),
                       (   )
                    ))
          
        # ReCalcula/Atualiza a inversa do Jacobiano numericamente
        Ji = np.linalg.pinv(J)  #Usar pseudoinversa para rejeitar erros de matrizes singulares
        
        # Aplica lei de controle qp = J(q)^-1 * (Xdp + K * (Xd - X)) 
        xdp = 0 # xd  constante, logo xdp = 0
        ydp = 0 # yd  constante, logo ydp = 0
        zdp = 0
        wxp = 0 
        wyp = 0 
        wzp = 0
        
        Xdp = np.array(( (xdp), #vetor de derivadas dos estados desejados
                         (ydp),
                         (zdp),
                         (wxp), 
                         (wyp),
                         (wzp)
                      ))
          
        K = np.array(( (20, 0, 0, 0, 0, 0), #matriz de ganhos
                       (0, 20, 0, 0, 0, 0), 
                       (0, 0, 20, 0, 0, 0),
                       (0, 0, 0, 20, 0, 0), 
                       (0, 0, 0, 0, 20, 0), 
                       (0, 0, 0, 0, 0, 20)
                    ))
          
        V =  Xdp + np.dot(K, E) #termo auxiliar V = (Xdp + K * (Xd - X))
          
        qp = np.dot(Ji, V) # qp = J(q)^-1 * (Xdp + K * (Xd - X)) 
          
        #Atualiza q pela integração numérica q_k+1 = q_k + qp * dt 
        q = q +  qp * dt
        q = np.arctan2(np.sin(q), np.cos(q)) # Conversão para o domínio [-pi +pi]
        
        isOK = 1
    else:
        isOK = 0
    
    return q[0], q[1], q[2], isOK

def cinematica_inv_num_Jt(xd, yd, zd, q0_m, q1_m, q2_m, dt):
    a0 = 0.3
    a1 = 0.5
    a2 = 0.5
    
    # Inicialização do vetor q
    q = np.array(( (q0_m),  
                   (q1_m),
                   (q2_m)
                ))
    
    r = np.sqrt(xd**2+yd**2+zd**2)
    
    if r >= a0-a1-a2 and r <= a0+a1+a2: # Workspace
        #Inicializa vetor Cinemática Direta
        Cd = np.array(( ( ) ),  
                        ( ) ),
                        ( ) )
                     ))

        E = np.array (( ( xd - Cd[0] ), #Inicializa vetor erro de posição
                        ( yd - Cd[1] ),
                        ( zd - Cd[2] ),
                        ( 0 ), #Ignora erro de rotação
                        ( 0 ),
                        ( 0 )
                    ))
        
        # ReCalcula/Atualiza o Jacobiano em função de q
        J = np.array(( (   ), 
                       (   ),
                       (   ),
                       (   ),
                       (   ),
                       (   )
                    ))
        
        # ReCalcula/Atualiza a inversa do Jacobiano numericamente
        Jt = np.transpose(J)
          
        K = np.array(( (20, 0, 0, 0, 0, 0), #matriz de ganhos
                       (0, 20, 0, 0, 0, 0), 
                       (0, 0, 20, 0, 0, 0),
                       (0, 0, 0, 20, 0, 0), 
                       (0, 0, 0, 0, 20, 0), 
                       (0, 0, 0, 0, 0, 20)
                    ))
          
        V =  np.dot(K, E) #termo auxiliar V = (Xdp + K * (Xd - X))
          
        qp = np.dot(Jt, V) # qp = J^T * (K * (Xd - X)) 
          
        #Atualiza q pela integração numérica q_k+1 = q_k + qp * dt 
        q = q +  qp * dt
        q = np.arctan2(np.sin(q), np.cos(q)) # Conversão para o domínio [-pi +pi]
        
        isOK = 1
    else:
        isOK = 0
    
    return q[0], q[1], q[2], isOK

def main():
    ############################################
    # Variáveis globais de simulação
    ############################################
    
    #Inicialização
    dt = 0.05
    
    #Controle do tempo de simulação
    t = 0
     
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter() #Controle de tempo
        t+=dt    
        # Obtém Pose do Alvo
        xd, yd, zd, phid, thetad, psid = Obter_Pose(AlvoHandle)
    
        x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
        
        #Aplica cinemática inversa
        #Captura posição angular corrente
        q0_m = Obter_Angulo_Junta(J0Handle)
        q1_m = Obter_Angulo_Junta(J1Handle)
        q2_m = Obter_Angulo_Junta(J2Handle)
        
        q0_r, q1_r, q2_r, isOK = cinematica_inv_num_Ji(xd, yd, zd, q0_m, q1_m, q2_m, dt)
    
        # Seta referência das juntas
        Setar_Juntas_Robo(q0_r, q1_r, q2_r)
        
        #Print de dados
        print("xd: %1.2f  yd: %1.2f  zd: %1.2f" % (xd, yd, zd))
        print(" x: %1.2f   y: %1.2f   z: %1.2f" % (x, y, z))
        
        if not(isOK):
            print("ERRO CINEMÁTICA INVERSA!")
        
        #Disparo do trigger de simulação
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        #Aguardando dt
        while(time.perf_counter()-t0 <= dt): _ # Loop de 50ms
try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)
