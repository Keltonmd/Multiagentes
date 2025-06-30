# Biblioteca para o Coppelia (versão nova com ZMQ API)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import matplotlib.pyplot as plt
import time

# Variveis globais de conexão
client = None
sim = None
simIK = None

# Variáveis globais
handleRobotOmni = None
handlePads = []
handleRobotUR5 = None
handleJointFanka = []
target = None
cubo = None

estanteAzul = None
estanteVermelha = None
posDisponivelAzul = {}
posDisponivelVer = {}

def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

def obterHandles():
    global handleRobotOmni, handleRobotUR5, handlePads, handleJointFanka, cubo, target

    handleRobotOmni = sim.getObject('/OmniPlatform')
    print(f"Robo Omni: {handleRobotOmni}")

    # Obter as juntas do Omni
    for i in range(4):
        junta = sim.getObject(f'/OmniPlatform/regularRotation{i + 1}')
        handlePads.append(junta)

    handleRobotUR5 = sim.getObject('/OmniPlatform/UR5')
    print(f"Robo UR5: {handleRobotUR5}")

    # Obter as juntas do UR5
    for i in range(7):
        junta = sim.getObject(f'/OmniPlatform/UR5/joint{i + 1}')
        handleJointFanka.append(junta)
    
    # Obter o target
    target = sim.getObject("/OmniPlatform/UR5/Target")
    cubo = sim.getObject("/Cuboid")

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
    sim.setJointTargetPosition(handleJointFanka[0], rad)

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

def moverTargetPara(x, y, z):
    sim.setObjectPosition(target, -1, [x, y, z])

# Teste
conectar()
obterHandles()

posicaoTarget = sim.getObjectPosition(target, -1)
print(f"Posicao: {posicaoTarget}")

posicaoCubo = sim.getObjectPosition(cubo, -1)
print(f"Posicao: {posicaoCubo}")

print("Movendo Garra para cubo")
moverTargetPara(posicaoCubo[0], posicaoCubo[1], posicaoCubo[2])

time.sleep(3)

# moverTargetPara(posicaoTarget[0], posicaoTarget[1], posicaoTarget[2])