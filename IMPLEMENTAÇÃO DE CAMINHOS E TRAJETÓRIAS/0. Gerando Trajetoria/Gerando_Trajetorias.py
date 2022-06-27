"""
 Autor: Prof. Lucas Vago Santana
 Curso: Robótica Industrial - Engenharia de Controle e Automação
 Instituição: Ifes - campus Linhares
 Revisado em: 22/05/2022
 
 # Modificação dos exemplos para controle direto da IDE Python
 # Utilize o PLAY e o CTRL+C do Spyder IDE para controlar a simulação sem necessidade de clicar nos botões do CoppeliaSim
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

# Frames da Simulação
_, FrameEHandle = vrep.simxGetObjectHandle(clientID,'FrameE',vrep.simx_opmode_oneshot_wait)
# Alvo
_, AlvoHandle = vrep.simxGetObjectHandle(clientID,'Alvo',vrep.simx_opmode_oneshot_wait)

#Função que obtém a pose de um objeto da cena
#Simula algoritmos de localização global [Ex. Visão Computacional]
def Obter_Pose(handle): 
    _, pos = vrep.simxGetObjectPosition(clientID, handle,-1,vrep.simx_opmode_oneshot_wait)
    _, ori = vrep.simxGetObjectOrientation(clientID, handle,-1,vrep.simx_opmode_oneshot_wait)
  
    x = pos[0]   
    y = pos[1]
    z = pos[2]
    phi = ori[0]
    theta = ori[1]
    psi = ori[2]
    
    return x, y, z, phi, theta, psi

# Retorna matriz de rotação de objetos da simulação
def Obter_R(phi, theta, psi):
  # Fonte: https://www.coppeliarobotics.com/helpFiles/en/eulerAngles.htm
  R = np.array((
                  (                              np.cos(psi)*np.cos(theta),                             -np.cos(theta)*np.sin(psi),           np.sin(theta) ),
                  ( np.cos(phi)*np.sin(psi) + np.cos(psi)*np.sin(phi)*np.sin(theta), np.cos(phi)*np.cos(psi) - np.sin(phi)*np.sin(psi)*np.sin(theta), -np.cos(theta)*np.sin(phi) ),
                  ( np.sin(phi)*np.sin(psi) - np.cos(phi)*np.cos(psi)*np.sin(theta), np.cos(psi)*np.sin(phi) + np.cos(phi)*np.sin(psi)*np.sin(theta),  np.cos(phi)*np.cos(theta) )
              ))
  return R

def Obter_q(R):
  n = 0.5 * np.sqrt(R[0][0] + R[1][1] + R[2][2] + 1)
  E = 0.5 * np.array ((
                             ( np.sign(R[2][1] - R[1][2]) * np.sqrt(R[0][0] - R[1][1] - R[2][2] + 1) ),
                             ( np.sign(R[0][2] - R[2][0]) * np.sqrt(R[1][1] - R[2][2] - R[0][0] + 1) ),
                             ( np.sign(R[1][0] - R[0][1]) * np.sqrt(R[2][2] - R[0][0] - R[1][1] + 1) )
                          ))
  q = np.array(( (n), (E[0]), (E[1]), (E[2]) ))
  
  return q
######################################################################################
################################  TRAJETÓRIA #########################################
######################################################################################

# Dados da Trajetória 
Xi = np.zeros(3)
Xf = np.zeros(3)
Xd = np.zeros(3)
Xdp =  np.zeros(3)

Qi = np.zeros(4)
Qf = np.zeros(4)
Qd = np.zeros(4)

def Iniciar_Planejador(x, y, z, phi, theta, psi, xd, yd, zd, phid, thetad, psid):
     # Geração da Trajetória 
    Xi = np.array((
        (x), # Posições iniciais
        (y),
        (z)
    ))
     
    Xf = np.array((
        (xd), # Posições finais
        (yd),
        (zd)
    ))
    
    Ri = Obter_R(phi, theta, psi)  # Matriz de Rotação Inicial
    Qi = Obter_q(Ri) #Quaternio que representa Ri
    
    
    Rf = Obter_R(phid, thetad, psid) #Matriz de Rotação Final
    Qf = Obter_q(Rf) #Quaternio que representa Rf
    
    return Xi, Xf, Qi, Qf
    
def Planejador_Trajetoria(Xi, Xf, Qi, Qf, tt, T):
    #Cálculo de coeficientes do polinômio
    S = np.array((
                    (0), #valor inicial s(0) = 0
                    (1), #valor final   s(T) = 1
                    (0), #valor inicial sp
                    (0), #valor final sp
                    (0),
                    (0)
            ))
    
    M = np.array((
                    (      0,        0,        0,      0,   0,      1),
                    (   T**5,     T**4,     T**3,   T**2,   T,      1),
                    (      0,        0,        0,      0,   1,      0),
                    ( 5*T**4,   4*T**3,   3*T**2,    2*T,   1,      0),
                    (      0,        0,        0,      2,   0,      0),
                    (20*T**3,  12*T**2,      6*T,      2,   0,      0)
            ))       

    C = np.dot(np.linalg.inv(M),S)

    s =    C[0] * tt**5 +    C[1] * tt**4 +    C[2] * tt**3 +    C[3] * tt**2 +  C[4] * tt +  C[5]; 
    sp = 5*C[0] * tt**4 +  4*C[1] * tt**3 +  3*C[2] * tt**2 +  2*C[3] * tt    +  C[4]; 

    Xd = (1-s)*Xi + s*Xf;  
    Xdp =(Xf-Xi)*sp;
    
    #Qd = (1-s)*Qi + s*Qf;  
    alpha = np.arccos(Qi[0]*Qf[0]+Qi[1]*Qf[1]+Qi[2]*Qf[2]+Qi[3]*Qf[3])
    Qd = ( np.sin((1-s)*alpha)*Qi + np.sin(s*alpha)*Qf )/np.sin(alpha)
    
    return Xd, Qd


def main():
    #Controle do tempo da simulação e da Trajetória Cartesiana
    dt = 0.05
    t = 0
    tt = 0
    
    # Obtém Pose do Alvo
    xd, yd, zd, phid, thetad, psid = Obter_Pose(AlvoHandle)
    
    # Obtém Pose do Efetuador Final
    x, y, z, phi, theta, psi = Obter_Pose(FrameEHandle)
    
    Xi, Xf, Qi, Qf = Iniciar_Planejador(x, y, z, phi, theta, psi, xd, yd, zd, phid, thetad, psid)
           
    tjt_isRunning = 1 # 1-Sim  0-Não (Supervisor no planejador de trajetória)
    
    while vrep.simxGetConnectionId(clientID) != -1:
        #Loop de controle do robô
        t0 = time.perf_counter() #Controle de tempo
        
        T = 1 # Período de interpoção em segundos    
          
        if tjt_isRunning == 1:
            Xd, Qd = Planejador_Trajetoria(Xi, Xf, Qi, Qf, tt, T)
            tt += dt
            vrep.simxSetObjectPosition(clientID,FrameEHandle, -1, Xd, vrep.simx_opmode_streaming)
            vrep.simxSetObjectQuaternion(clientID,FrameEHandle, -1, [Qd[1], Qd[2], Qd[3], Qd[0]] , vrep.simx_opmode_streaming)
            
            if tt>= T: #Trajetória finalizada
                tt = 0
                tjt_isRunning = 0
                print("Fim") #Printa
            
        #Disparo do trigger de simulação
        vrep.simxSynchronousTrigger(clientID); # Trigger next simulation step (Blocking function call)
        
        t += dt
        #Aguardando dt
        while(time.perf_counter()-t0 <= dt): _ # Loop de 50ms


try:
    main()
    
except KeyboardInterrupt:
    # stop the simulation:
    vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking)

    # Now close the connection to CoppeliaSim:
    vrep.simxFinish(clientID)