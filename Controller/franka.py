from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time

client = None
sim = None
simIK = None

handleFrankaBase = None
handleFrankaJoints = []
cubo = None
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

def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

def obterHandles():
    global handleFrankaBase, handleFrankaJoints, cubo, target
    global handleActive1, handleActive2, simBase

    handleFrankaBase = sim.getObject('/Franka')

    for i in range(7):
        junta = sim.getObject('/Franka/joint', {'index': i})
        handleFrankaJoints.append(junta)
        
    cubo = sim.getObject("/Cuboid")
    target = sim.getObject("/Franka/target")
    
    # Handle da Garra
    simBase = sim.getObject("/Franka/ROBOTIQ85")
    handleActive1 = sim.getObject("/Franka/ROBOTIQ85/active1")
    handleActive2 = sim.getObject("/Franka/ROBOTIQ85/active2")

    print(f"handleFrankaBase: {handleFrankaBase}, cubo: {cubo}, handleFrankaJoints: {handleFrankaJoints}")

def moverTargetAcimaDoBloco(altura=0.2):
    pos_cubo = sim.getObjectPosition(cubo, -1)  # global
    pos_target = [pos_cubo[0], pos_cubo[1], pos_cubo[2] + altura]
    
    sim.setObjectPosition(target, -1, pos_target)
    
    print(f"[INFO] Target movido para {pos_target} com orientação para baixo.")

def ikGarra():
    global simTip1, simTip2, ikEnv, ikGroup1, ikGroup2, SimTarget1, SimTarget2
    
    ikEnv = simIK.createEnvironment()
    
    ikGroup1 = simIK.createGroup(ikEnv)
    simTip1 = sim.getObject("/Franka/ROBOTIQ85/LclosureDummyA")
    SimTarget1 = sim.getObject("/Franka/ROBOTIQ85/LclosureDummyB")
    simIK.addElementFromScene(ikEnv, ikGroup1, simBase, simTip1, SimTarget1, simIK.constraint_x+simIK.constraint_z)
    
    ikGroup2 = simIK.createGroup(ikEnv)
    simTip2 = sim.getObject("/Franka/ROBOTIQ85/RclosureDummyA")
    SimTarget2 = sim.getObject("/Franka/ROBOTIQ85/RclosureDummyB")
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
    
    print("[INFO] Abrindo garra...")

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

    print("[INFO] Fechando garra...")
 
def subirBraco(z_final, velocidade = 0.001, intervalo = 0.001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual < z_final:
        z_atual += velocidade
        
        if z_atual > z_final: 
            z_atual = z_final
        
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def descerBraco(z_final, velocidade = 0.001, intervalo = 0.001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual > z_final:
        z_atual -= velocidade
        
        if z_atual < z_final: 
            z_atual = z_final
        
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)
    
def alinharComObjeto(obj_path):
    pos_alvo = sim.getObjectPosition(sim.getObject(obj_path), -1)
    pos_atual = sim.getObjectPosition(target, -1)
    
    nova_pos = [pos_alvo[0], pos_alvo[1], pos_atual[2]]
    sim.setObjectPosition(target, -1, nova_pos)
    time.sleep(0.1)

def pegaBlocoEsteira():
    bloco = "/esteiraColeta"
    altura = 0.02
    
    # Alinha o target horizontalmente com o bloco
    alinharComObjeto(bloco)
    
    # z Final
    z_bloco = sim.getObjectPosition(sim.getObject(bloco), -1)[2] + altura
    descerBraco(z_bloco,)
    
    #Implementar o Fecha Garra
    fecharGarra()
    time.sleep(2)
    # Sobe de volta à altura anterior (posição de início do movimento)
    altura_inicial = sim.getObjectPosition(sim.getObject("/pontoEspera"), -1)[2]
    subirBraco(altura_inicial)

def moverBraco(x_alvo, y_alvo, velocidade = 0.001, intervalo = 0.001):
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
            
def entregaBloco():
    pos_Entrega = sim.getObjectPosition(sim.getObject("/youBot/cuboPos"), -1)
    
    moverBraco(pos_Entrega[0], pos_Entrega[1])
    descerBraco(pos_Entrega[2])
    time.sleep(1)
    abrirGarra()
    time.sleep(0.5)
    
    posEspera = sim.getObjectPosition(sim.getObject("/pontoEspera"), -1)
    subirBraco(posEspera[2])
    moverBraco(posEspera[0], posEspera[1])
    

# Executa
conectar()
obterHandles()
ikGarra()
abrirGarra()
pegaBlocoEsteira()
entregaBloco()
#abrirGarra()
#fecharGarra()