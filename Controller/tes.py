# Biblioteca para o Coppelia (versão nova com ZMQ API)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

# Variveis globais de conexão
client = None
sim = None
simIK = None

# Variáveis globais
handleRobotUR10 = None
handleJointUR10 = []
target = None
cubo = None

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
estante = None

def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
def obterHandles():
    global handleRobotUR10, handleJointUR10, cubo, target
    global handleActive1, handleActive2, estante

    handleRobotUR10 = sim.getObject('/UR10')
    print(f"Robo UR10: {handleRobotUR10}")

    # Obter as juntas do UR10
    for i in range(6):
        junta = sim.getObject(f'/UR10/joint', {'index': i})
        handleJointUR10.append(junta)
    
    # Obter o target
    target = sim.getObject("/UR10/target")
    handleActive1 = sim.getObject("/UR10/ROBOTIQ85/active1")
    handleActive2 = sim.getObject("/UR10/ROBOTIQ85/active2")
    
    estante = sim.getObject("/Estante1/teste")
    cubo = sim.getObject("/youBot/cuboPos")
    
    
def ikGarra():
    global simTip1, simTip2, ikEnv, ikGroup1, ikGroup2, SimTarget1, SimTarget2, simBase

    simBase = sim.getObject("/UR10/ROBOTIQ85")  # Ajuste o path se necessário

    ikEnv = simIK.createEnvironment()

    ikGroup1 = simIK.createGroup(ikEnv)
    simTip1 = sim.getObject("/UR10/ROBOTIQ85/LclosureDummyA")
    SimTarget1 = sim.getObject("/UR10/ROBOTIQ85/LclosureDummyB")
    simIK.addElementFromScene(ikEnv, ikGroup1, simBase, simTip1, SimTarget1, simIK.constraint_x+simIK.constraint_z)

    ikGroup2 = simIK.createGroup(ikEnv)
    simTip2 = sim.getObject("/UR10/ROBOTIQ85/RclosureDummyA")
    SimTarget2 = sim.getObject("/UR10/ROBOTIQ85/RclosureDummyB")
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
        sim.setJointTargetVelocity(handleActive1, -0.05)
        sim.setJointTargetVelocity(handleActive2, -0.08)
    else:
        sim.setJointTargetVelocity(handleActive1, -0.08)
        sim.setJointTargetVelocity(handleActive2, -0.08)

    simIK.handleGroup(ikEnv, ikGroup1, {"syncWorlds": True})
    simIK.handleGroup(ikEnv, ikGroup2, {"syncWorlds": True})

def moverBraco(x_alvo, y_alvo, z_alvo, velocidade=0.001, intervalo=0.001):
    nova_pos = sim.getObjectPosition(target, -1)
    x_atual = nova_pos[0]
    y_atual = nova_pos[1]
    z_atual = nova_pos[2]
    while x_atual != x_alvo or y_atual != y_alvo or z_atual != z_alvo:
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
        if z_atual != z_alvo:
            if z_atual < z_alvo:
                z_atual += velocidade
                if z_atual > z_alvo:
                    z_atual = z_alvo
            else:
                z_atual -= velocidade
                if z_atual < z_alvo:
                    z_atual = z_alvo
            
        nova_pos[0] = x_atual
        nova_pos[1] = y_atual
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def rotacionar(velocidade=0.5, intervalo=1):
    nova_orientacao = sim.getObjectOrientation(target, -1)
    ang_atual = np.around(np.degrees(nova_orientacao))[2]
    print(f"Angulo atual: {ang_atual}\n Lista do euller: {nova_orientacao}")
    if ang_atual == 0:
        frente = True
    else:
        frente = False
    
    while True:
        if frente:
            ang_atual += velocidade
            print(ang_atual)
            
            if ang_atual >= 180:
                ang_atual = -180
                nova_orientacao[2] = np.radians(ang_atual)
                sim.setObjectOrientation(target, -1, nova_orientacao)
                break
        
            nova_orientacao[2] = np.radians(ang_atual)
            sim.setObjectOrientation(target, -1, nova_orientacao)
            time.sleep(intervalo)
        else:
            ang_atual += velocidade
            
            if ang_atual >= 0:
                ang_atual = 0
                nova_orientacao[2] = np.radians(ang_atual)
                sim.setObjectOrientation(target, -1, nova_orientacao)
                break
        
            nova_orientacao[2] = np.radians(ang_atual)
            sim.setObjectOrientation(target, -1, nova_orientacao)
            time.sleep(intervalo)
    
def descerBraco(z_final, velocidade = 0.001, intervalo = 0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual > z_final:
        z_atual -= velocidade
        
        if z_atual < z_final: 
            z_atual = z_final
        
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)
   
def subirBraco(z_final, velocidade=0.0005, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual < z_final:
        z_atual += velocidade
        if z_atual > z_final:
            z_atual = z_final
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def pegarBloco():
    
    time.sleep(1)
    
    # Pega a posição do cubo que está no youBot
    pos_cubo = sim.getObjectPosition(cubo, -1)
    
    if pos_cubo[1] > 0:
        pos_cubo[1] -= 0.025
    else:
        pos_cubo[1] += 0.025

    # Move o braço horizontalmente para a posição do cubo
    moverBraco(pos_cubo[0], pos_cubo[1], pos_cubo[2])

    time.sleep(1)
    # Fecha a garra para pegar o bloco
    fecharGarra()
    time.sleep(2)

    # Sobe o braço até a posição de espera
    posEspera = sim.getObjectPosition(sim.getObject("/UR10/posEspera"), -1)
    subirBraco(posEspera[2])

    # Move para a posição de espera
    moverBraco(posEspera[0], posEspera[1], posEspera[2])

def guardarBloco():
    
    orient = sim.getObjectOrientation(handleRobotUR10, -1)
    orient[2] = -1 * np.radians(180)
    sim.setObjectOrientation(handleRobotUR10, -1, orient)
    
    pos = sim.getObjectPosition(estante, -1)
    
    descerBraco(pos[2])
    
    moverBraco(pos[0], pos[1], pos[2])
    
    time.sleep(1)
    abrirGarra()
    time.sleep(1)
    
    # Sobe o braço até a posição de espera
    posEspera = sim.getObjectPosition(sim.getObject("/UR10/posEspera"), -1)
    
    moverBraco(posEspera[0], posEspera[1], pos[2])
    
    subirBraco(posEspera[2])
    

conectar()
obterHandles()
ikGarra()
abrirGarra()
pegarBloco()
#guardarBloco()


