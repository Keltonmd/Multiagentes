# Biblioteca para o Coppelia (versão nova com ZMQ API)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import matplotlib.pyplot as plt
import time

# Variveis globais de conexão
client = None
sim = None

# Variáveis globais
handleRobotOmni = None
handleRobotUR5 = None
handlePads = []
handleJointUR5 = []

def conectar():
    global client, sim
    client = RemoteAPIClient()
    sim = client.require('sim')

def obterHandles():
    global handleRobotOmni, handleRobotUR5, handlePads, handleJointUR5

    #handleRobotOmni = sim.getObject('/OmniPlatform')
    #print(f"Robo Omni: {handleRobotOmni}")

    handleRobotUR5 = sim.getObject('/UR5')
    print(f"Robo UR5: {handleRobotUR5}")

    # Obter as juntas do Omni
    #for i in range(4):
    #    junta = sim.getObject(f'/OmniPlatform/regularRotation{i + 1}')
    #    handlePads.append(junta)

    # Obter as juntas do UR5
    for i in range(6):
        junta = sim.getObject(f'/UR5/jointUR{i + 1}')
        handleJointUR5.append(junta)

# Controlar o OmniPlataform
def irparaFT(v):
    # V positivo Frente, -v Tras
    sim.setJointTargetVelocity(handlePads[0], v)
    sim.setJointTargetVelocity(handlePads[1], v)
    sim.setJointTargetVelocity(handlePads[2], v)
    sim.setJointTargetVelocity(handlePads[3], v)

def irParaL(v):
    # V positivo direjta, -v esquerda
    sim.setJointTargetVelocity(handlePads[0], v)
    sim.setJointTargetVelocity(handlePads[1], v)
    sim.setJointTargetVelocity(handlePads[2], -v)
    sim.setJointTargetVelocity(handlePads[3], -v)

def irParaDD(v):
    # +v Diagnoal Cima, -v Diagnoal baixo
    sim.setJointTargetVelocity(handlePads[0], 0)
    sim.setJointTargetVelocity(handlePads[1], v)
    sim.setJointTargetVelocity(handlePads[2], 0)
    sim.setJointTargetVelocity(handlePads[3], -v)

def irParaDE(v):
    # +v Diagnoal Cima, -v Diagnoal baixo
    sim.setJointTargetVelocity(handlePads[0], -v)
    sim.setJointTargetVelocity(handlePads[1], 0)
    sim.setJointTargetVelocity(handlePads[2], v)
    sim.setJointTargetVelocity(handlePads[3], 0)

def girarOmni(v):
    # +v gira sentido horario, v = 0 para o robo, -v gira sentido anti horario
    sim.setJointTargetVelocity(handlePads[0], v)
    sim.setJointTargetVelocity(handlePads[1], v)
    sim.setJointTargetVelocity(handlePads[2], v)
    sim.setJointTargetVelocity(handlePads[3], v)
    
# Controlar o UR5
def girarBaseUr5(anguloGraus):
    rad = np.radians(anguloGraus)
    sim.setJointTargetPosition(handleJointUR5[0], rad)

# Calcular o Angulo pelo Euller
def calcularRotacao(alpha, beta, gamma):
    # Quando alpha e beta são diferentes de 0
    
    # Casos fixos
    if (alpha, beta, gamma) == (0, 90, 180):
        return 0, (0 * (np.pi / 180))
    elif (alpha, beta, gamma) == (-90, 0, -90):
        return 90, (90 * (np.pi / 180))
    elif (alpha, beta, gamma) == (0, -90, 0):
        return 180, (180 * (np.pi / 180))
    elif (alpha, beta, gamma) == (90, 0, 90):
        return 270, (270 * (np.pi / 180))
    
    # Casos restantes:
    if alpha == -90 and gamma == -90 and beta > 0:
        return 90 - beta, ((90 - beta) * (np.pi / 180))
    elif alpha == -90 and gamma == -90 and beta < 0:
        return 90 + (beta * -1), ((90 + (beta * -1)) * (np.pi / 180))
    elif alpha == 90 and gamma == 90 and beta < 0:
        return 180 + (90 - (beta * -1)), ((180 + (90 - (beta * -1))) * (np.pi / 180))
    elif alpha == 90 and gamma == 90 and beta > 0:
        return 270 + beta, ((270 + beta) * (np.pi / 180))
    
    # Quando alpha e beta são 0
    
    if (alpha, beta, gamma) == (0, 0, 0):
        return 0, (0 * (np.pi / 180))
    elif (alpha, beta, gamma) == (0, 0, 90):
        return 90, (90 * (np.pi / 180))
    elif (alpha, beta, gamma) == (0, 0, -180) or (alpha, beta, gamma) == (0, 0, 180):
        return 180, (180 * (np.pi / 180))
    elif (alpha, beta, gamma) == (0, 0, -90):
        return 270, (270 * (np.pi / 180))
    
    if alpha == 0 and beta == -0 and gamma > 0 and gamma < 180:
        return gamma, (gamma * (np.pi / 180))
    elif alpha == 0 and beta == 0 and gamma < 0:
        return 180 + (180 - (gamma * -1)), ((180 + (180 - (gamma * -1))) * (np.pi / 180))
    
    return -1, -1 

def obterOrientacao(objeto):
    orientacao = np.array(sim.getObjectOrientation(objeto, -1))
    orientacao = orientacao * 180/np.pi
    orientacao = np.around(orientacao, decimals=0)
    ang, rad = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
    return ang, rad

# Teste
conectar()
obterHandles()

# Girar o UR5 para a posição zero
girarBaseUr5(0)
time.sleep(2)

# Pegar a orientação do eixo 5
ang, rad = obterOrientacao(handleJointUR5[4])
print("Orientação da junta 5:", ang)

# Girar base para 90 graus
girarBaseUr5(90)
time.sleep(2)

# Verificar orientação após movimento
ang, rad = obterOrientacao(handleJointUR5[4])
print("Nova orientação da junta 5:", ang)

# Encerrar simulação (opcional)
# sim.stopSimulation()
