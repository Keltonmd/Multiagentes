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
    
def pegaBlocoEsteira(velocidade = 0.001, intervalo = 0.0001):
    # Pega o handle e posição do bloco da esteira
    bloco = sim.getObject("/esteiraColeta")
    pos_bloco = sim.getObjectPosition(bloco, -1)
    
    # Posição atual do target (ponta do braço)
    pos_target = sim.getObjectPosition(target, -1)
    
    # Alinha X e Y do target com o bloco (mantém altura atual)
    nova_pos = [pos_bloco[0], pos_bloco[1], pos_target[2]]
    sim.setObjectPosition(target, -1, nova_pos)
    
    altura_final = pos_bloco[2] + 0.02
    z_atual = nova_pos[2]
    while z_atual > altura_final:
        z_atual -= velocidade
        
        if z_atual < altura_final:
            z_atual = altura_final
        
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
        
        time.sleep(intervalo)
    
    print(f"[INFO] Target alinhado e posicionado sobre a esteira em z = {altura_final:.3f}")

def levantarBlocoEsteria():
    # Pega o handle e posição do bloco da esteira
    bloco = sim.getObject("/pontoEspera")
    pos_bloco = sim.getObjectPosition(bloco, -1)
    

# Executa
conectar()
obterHandles()
ikGarra()
pegaBlocoEsteira()
#abrirGarra()
#fecharGarra()