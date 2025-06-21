import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient('localhost', 23000)
print("Conectado!")
sim = client.require('sim')
simIK = client.require('simIK')

#handles
UR5 = sim.getObject('/UR5')
print("UR5 ok")
ROBOTIQ85 = sim.getObject('/UR5/ROBOTIQ85')
print("ROBOTIQ85 ok")
junta1 = sim.getObject('/UR5/ROBOTIQ85/active1')
print("junta1 ok")
junta2 = sim.getObject('/UR5/ROBOTIQ85/active2')
print("junta2 ok")
cuboid = sim.getObject('/Cuboid')
print("Cuboid ok")

#IK garra
ikEnv = None
ikGrupo1 = None
ikGrupo2 = None

#Garra
def grupoIKGarra():
    global ikEnv, ikGrupo1, ikGrupo2

    try:
        ikEnv = simIK.createEnvironment() #onde irá ser feito os cálculos
        if ikEnv == -1:
            raise Exception("Falha ao criar ambiente IK")
        ikGrupo1 = simIK.createGroup(ikEnv) #dedo 1
        ikGrupo2 = simIK.createGroup(ikEnv) #dedo 2
        if ikGrupo1 == -1 or ikGrupo2 == -1:
            raise Exception("Falha ao criar grupos IK")
    
        #handles para fazer grupos do IK
        base = sim.getObject('/ROBOTIQ85')
        print("base ok", base)
        tipGarra1 = sim.getObject('/UR5/ROBOTIQ85/LclosureDummyA')
        print("tipGarra1 ok")
        targetGarra1 = sim.getObject('/UR5/ROBOTIQ85/LclosureDummyB')
        print("targetGarra1 ok")
        tipGarra2 = sim.getObject('/UR5/ROBOTIQ85/RclosureDummyA')
        print("tipGarra2 ok")
        targetGarra2 = sim.getObject('/UR5/ROBOTIQ85/RclosureDummyB')
        print("targetGarra2 ok")

        #criando grupos IK
        result1 = simIK.addElementFromScene(ikEnv, ikGrupo1, base, tipGarra1, targetGarra1, simIK.constraint_x + simIK.constraint_z)
        result2 = simIK.addElementFromScene(ikEnv, ikGrupo2, base, tipGarra2, targetGarra2, simIK.constraint_x + simIK.constraint_z)
        if result1 == -1 or result2 == -1:
            print("erro ao criar grupos IK")
            
        print("grupos IK criados com sucesso!")
    except Exception as e:
        print('erro ik ', e)

def limparIK():
    global ikEnv
    if ikEnv:
        simIK.eraseEnvironment(ikEnv)
        print('limpo')

def garraAbrirFechar(j1, j2, fechando):
    global ikEnv, ikGrupo1, ikGrupo2
    p1 = sim.getJointPosition(j1) 
    p2 = sim.getJointPosition(j2)

    if fechando:
        if p1 < p2 - 0.008:
            sim.setJointTargetVelocity(j1, -0.01)
            sim.setJointTargetVelocity(j2, -0.04)
        else:
            sim.setJointTargetVelocity(j1, -0.04)
            sim.setJointTargetVelocity(j2, -0.04)
    else: # abrindo
        if p1 < p2:
            sim.setJointTargetVelocity(j1, 0.04)
            sim.setJointTargetVelocity(j2, 0.02)
        else:
            sim.setJointTargetVelocity(j1, 0.02)
            sim.setJointTargetVelocity(j2, 0.04)
            
    # atualizar posição
    if ikEnv and ikGrupo1 and ikGrupo2:
        simIK.handleGroup(ikEnv, ikGrupo1, {'syncWorlds': True})
        simIK.handleGroup(ikEnv, ikGrupo2, {'syncWorlds': True})
        
grupoIKGarra()  # inicializa os grupos IK
time.sleep(0.5)

# fechar a garra
garraAbrirFechar(junta1, junta2, True)
time.sleep(2)

#abrir a garra
garraAbrirFechar(junta1, junta2, False)
time.sleep(2)

limparIK()