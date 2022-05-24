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
_, J1Handle = vrep.simxGetObjectHandle(clientID,'J1',vrep.simx_opmode_oneshot_wait)
_, J2Handle = vrep.simxGetObjectHandle(clientID,'J2',vrep.simx_opmode_oneshot_wait)

# Frame da Ferramenta
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)

# Alvo
_, AlvoHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

# Envia referência de ângulo para juntas
def Setar_Juntas_Robo(q1, q2): 
   #Aplica cálculos ao objeto no simulador
   vrep.simxSetJointTargetPosition(clientID, J1Handle, q1, vrep.simx_opmode_streaming)
   vrep.simxSetJointTargetPosition(clientID, J2Handle, q2, vrep.simx_opmode_streaming)

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

def main():
    #Inicialização
    dt = 0.05
    
    #Controle do tempo de simulação
    t = 0
    
    #Dimensões dos elos do robô
    a1 = 0.5
    a2 = 0.5
    
    #Loop de controle do robô
    while vrep.simxGetConnectionId(clientID) != -1:
        t0 = time.perf_counter() #Controle de tempo
        t+=dt
      
        # Obtém Pose do Alvo
        x_a, y_a, z_a, phi_a, theta_a, psi_a = Obter_Pose(AlvoHandle)
    
        x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
        
        #Captura posição angular corrente
        q1_m = Obter_Angulo_Junta(J1Handle)
        q2_m = Obter_Angulo_Junta(J2Handle)
        
        #Programa aqui a cinemática inversa
        
        def cinematica_inv(xd, yd, a1, a2, config):
          r = np.sqrt(xd**2 + yd**2) 
          if r >=a1-a2 and r<=a1+a2: 
            if config == 1:
              q2 = np.arccos((xd**2+yd**2-a1**2-a2**2)/(2*a1*a2))
              q1 = np.arctan2(yd,xd) - np.arctan2((a2*np.sin(q2)),(a1+a2*np.cos(q2)))
            else:
              q2 = -np.arccos((xd**2+yd**2-a1**2-a2**2)/(2*a1*a2))
              q1 = np.arctan2(yd,xd) - np.arctan2((a2*np.sin(q2)),(a1+a2*np.cos(q2)))
           
            #q1=np.degrees(q1)
            #q2=np.degrees(q2)
            
            q1_r = q1
            q2_r = q2
            
            Setar_Juntas_Robo(q1_r, q2_r)
            
            print("\nCinemática inversa resolvida!")
            print("Solução de q1: %1.2f°" % np.degrees(q1))
            print("Solução de q2: %1.2f°" % np.degrees(q2))
          else:
            print("ERRO!")
            print("A cinemática inversa NÃO foi resolvida")
            print("Certifique-se que é possível alcançar (xd, yd)!")
        cinematica_inv(x_a, y_a, a1, a2, config=1)
        
        # Seta referência das juntas
        
        
        #Print de dados
        print("\nALVO: (%1.2f, %1.2f, %1.2f)" % (x_a, y_a, z_a))
        print("ATUADOR: (%1.2f, %1.2f, %1.2f)" % (x, y, z))
        #print("phi: %1.2f  theta: %1.2f  psi: %1.2f" % (phi*180/np.pi, theta*180/np.pi, psi*180/np.pi))
        #print("q1: %1.2f  q2: %1.2f" % (q1_m*180/np.pi,q2_m*180/np.pi))
        
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