# Biblioteca para o Coppelia
import sim
# Bibliotecas uteis
import numpy as np
import matplotlib.pyplot as plt
import time

# coordenada para recebimento: X -2.550, y: 1.350

# Variaveis globais
ip = '127.0.0.1'
port = 19999
clientId = None

handleRobotOmni = None
handleRobotUR5 = None
handlePads = []
handleJointUR5 = []

def conectar():
    global clientId
    
    # Fechar conecões
    sim.simxFinish(-1)
    
    # Conectar com o Coppelia
    clientId = sim.simxStart(ip, port, True, True, 5000, 5)
    
    if clientId != 1:
        print("Conexão via API realizada com sucesso")

def obterHandles():
    global handleRobotOmni, handlePads, handleJointUR5, handleRobotUR5
    # Obter o robo Omni
    returnCode, handleRobotOmni = sim.simxGetObjectHandle(clientId, "OmniPlatform", sim.simx_opmode_oneshot_wait)
    print(handleRobotOmni)
    
    # Obter o robo UR5
    returnCode, handleRobotUR5 = sim.simxGetObjectHandle(clientId, "UR5", sim.simx_opmode_oneshot_wait)
    print(handleRobotUR5)
    
    # Obter as juntas do robo Omni
    for i in range(5):
        _, junta = sim.simxGetObjectHandle(clientId,f"regularRotation{i + 1}", sim.simx_opmode_oneshot_wait)
        print(junta)
        handlePads.append(junta)
    
    # Obter as Juntas do robo UR5
    for i in range(6):
        _, junta = sim.simxGetObjectHandle(clientId,f"jointUR{i + 1}", sim.simx_opmode_oneshot_wait)
        print(junta)
        handleJointUR5.append(junta)

def irParaFT(v):
    # V positivo Frente, -v Tras
    sim.simxSetJointTargetVelocity(clientId, handlePads[0], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[1], -v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[2], -v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[3], v, sim.simx_opmode_streaming + 5)
    
def irParaL(v):
    # V positivo direjta, -v esquerda
    sim.simxSetJointTargetVelocity(clientId, handlePads[0], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[1], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[2], -v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[3], -v, sim.simx_opmode_streaming + 5)

def irParaDD(v):
    # +v Diagnoal Cima, -v Diagnoal baixo
    sim.simxSetJointTargetVelocity(clientId, handlePads[0], 0, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[1], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[2], 0, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[3], -v, sim.simx_opmode_streaming + 5)
    
def irParaDE(v):
    # +v Diagnoal Cima, -v Diagnoal baixo
    sim.simxSetJointTargetVelocity(clientId, handlePads[0], -v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[1], 0, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[2], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[3], 0, sim.simx_opmode_streaming + 5)

def girarRobo(v):
    # +v gira sentido horario, v = 0 para o robo, -v gira sentido anti horario
    sim.simxSetJointTargetVelocity(clientId, handlePads[0], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[1], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[2], v, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handlePads[3], v, sim.simx_opmode_streaming + 5)


conectar()
obterHandles()

v = 80 * 2.398795 * np.pi / 180 # Velocidade do Omni
v = v * -1

sim.simxSetJointTargetVelocity(clientId, handleJointUR5[0], 5, sim.simx_opmode_oneshot_wait)