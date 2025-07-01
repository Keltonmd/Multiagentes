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
handleJointUR5 = []
target = None

# Garra Robotiq85
handleActive1 = None
handleActive2 = None

# Requisitos do IK
ikEnv = None
simBase = None

ikGroup1 = None
simTip1 = None
SimTarget1 = None

ikGroup2 = None
simTip2 = None
SimTarget2 = None

# Estante
estanteAzul = None
estanteVermelha = None
posDisponivelAzul = []
posDisponivelVer = []

def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

def obterHandles():
    global handleRobotOmni, handleRobotUR5, handlePads, handleJointUR5, cubo, target
    global posDisponivelAzul, posDisponivelVer, handleActive1, handleActive2

    handleRobotOmni = sim.getObject('/OmniPlatform')
    print(f"Robo Omni: {handleRobotOmni}")

    # Obter as juntas do Omni
    for i in range(4):
        junta = sim.getObject(f'/OmniPlatform/regularRotation{i + 1}')
        handlePads.append(junta)

    handleRobotUR5 = sim.getObject('/OmniPlatform/UR5')
    print(f"Robo UR5: {handleRobotUR5}")

    # Obter as juntas do UR5
    for i in range(6):
        junta = sim.getObject(f'/OmniPlatform/UR5/jointUR{i + 1}')
        handleJointUR5.append(junta)
    
    # Obter o target
    target = sim.getObject("/OmniPlatform/UR5/target")
    handleActive1 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/active1")
    handleActive2 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/active2")
    
    # Handle da Estante
    for i in range(10):
        pos = sim.getObjectPosition(sim.getObject('/Estante1/pos', {'index': i}), -1)
        posDisponivelAzul.append({'pos': pos, 'livre': True})
    
    for i in range(10):
        pos = sim.getObjectPosition(sim.getObject('/Estante2/pos', {'index': i}), -1)
        posDisponivelVer.append({'pos': pos, 'livre': True})

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

def ikGarra():
    global simTip1, simTip2, ikEnv, ikGroup1, ikGroup2, SimTarget1, SimTarget2, simBase

    simBase = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85")  # Ajuste o path se necessário

    ikEnv = simIK.createEnvironment()

    ikGroup1 = simIK.createGroup(ikEnv)
    simTip1 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/LclosureDummyA")
    SimTarget1 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/LclosureDummyB")
    simIK.addElementFromScene(ikEnv, ikGroup1, simBase, simTip1, SimTarget1, simIK.constraint_x+simIK.constraint_z)

    ikGroup2 = simIK.createGroup(ikEnv)
    simTip2 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/RclosureDummyA")
    SimTarget2 = sim.getObject("/OmniPlatform/UR5/ROBOTIQ85/RclosureDummyB")
    simIK.addElementFromScene(ikEnv, ikGroup2, simBase, simTip2, SimTarget2, simIK.constraint_x+simIK.constraint_z)

def abrirGarra():
    p1 = sim.getJointPosition(handleActive1)
    p2 = sim.getJointPosition(handleActive2)

    if p1 < p2:
        sim.setJointTargetVelocity(handleActive1, 0.4)
        sim.setJointTargetVelocity(handleActive2, 0.2)
    else:
        sim.setJointTargetVelocity(handleActive1, 0.2)
        sim.setJointTargetVelocity(handleActive2, 0.4)
    
    simIK.handleGroup(ikEnv, ikGroup1, {"syncWorlds": True})
    simIK.handleGroup(ikEnv, ikGroup2, {"syncWorlds": True})

def fecharGarra():
    p1 = sim.getJointPosition(handleActive1)
    p2 = sim.getJointPosition(handleActive2)

    if p1 < p2 - 0.008:
        sim.setJointTargetVelocity(handleActive1, -0.01)
        sim.setJointTargetVelocity(handleActive2, -0.04)
    else:
        sim.setJointTargetVelocity(handleActive1, -0.04)
        sim.setJointTargetVelocity(handleActive2, -0.04)

    simIK.handleGroup(ikEnv, ikGroup1, {"syncWorlds": True})
    simIK.handleGroup(ikEnv, ikGroup2, {"syncWorlds": True})

def subirBraco(z_final, velocidade=0.001, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual < z_final:
        z_atual += velocidade
        if z_atual > z_final:
            z_atual = z_final
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def descerBraco(z_final, velocidade=0.001, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual > z_final:
        z_atual -= velocidade
        if z_atual < z_final:
            z_atual = z_final
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def moverBraco(x_alvo, y_alvo, velocidade=0.001, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    x_atual = nova_pos[0]
    y_atual = nova_pos[1]
    while x_atual != x_alvo or y_atual != y_alvo:
        if x_atual != x_alvo:
            if x_atual < x_alvo:
                x_atual += velocidade
                if x_atual > x_alvo:
                    x_atual = x_alvo
            else:
                x_atual -= velocidade
                if x_atual < x_alvo:
                    x_atual = x_alvo
        if y_atual != y_alvo:
            if y_atual < y_alvo:
                y_atual += velocidade
                if y_atual > y_alvo:
                    y_atual = y_alvo
            else:
                y_atual -= velocidade
                if y_atual < y_alvo:
                    y_atual = y_alvo
        nova_pos[0] = x_atual
        nova_pos[1] = y_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def pegarBloco():
    # Pega a posição do cubo que está no youBot
    pos_cubo = sim.getObjectPosition(sim.getObject("/youBot/cuboPos"), -1)

    # Move o braço horizontalmente para a posição do cubo
    moverBraco(pos_cubo[0], pos_cubo[1])

    # Desce até a altura do cubo (ajuste caso o braço precise descer mais)
    descerBraco(pos_cubo[2])

    # Fecha a garra para pegar o bloco
    fecharGarra()
    time.sleep(0.5)

    # Sobe o braço até a posição de espera
    posEspera = sim.getObjectPosition(sim.getObject("/OmniPlatform/UR5/posEspera"), -1)
    subirBraco(posEspera[2])

    # Move para a posição de espera
    moverBraco(posEspera[0], posEspera[1])
    

# Teste
conectar()
obterHandles()
ikGarra()
abrirGarra()