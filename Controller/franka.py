from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time
import threading
import paho.mqtt.client as mqtt

client = None
sim = None
simIK = None
mqtt_client = None

handleFrankaBase = None
handleFrankaJoints = []
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

# Variaveis para colaboração Memoria do Agente
espera_bloco = False
destino_livre = False
segurando_bloco = False
finalizado = False

def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')

def obterHandles():
    global handleFrankaBase, handleFrankaJoints, target
    global handleActive1, handleActive2, simBase

    handleFrankaBase = sim.getObject('/Franka')

    for i in range(7):
        junta = sim.getObject('/Franka/joint', {'index': i})
        handleFrankaJoints.append(junta)
        
    target = sim.getObject("/Franka/target")
    
    # Handle da Garra
    simBase = sim.getObject("/Franka/ROBOTIQ85")
    handleActive1 = sim.getObject("/Franka/ROBOTIQ85/active1")
    handleActive2 = sim.getObject("/Franka/ROBOTIQ85/active2")

    print(f"handleFrankaBase: {handleFrankaBase}, handleFrankaJoints: {handleFrankaJoints}")

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

# Capacidades do Agente

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
 
def subirBraco(z_final, velocidade = 0.001, intervalo = 0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    z_atual = nova_pos[2]
    while z_atual < z_final:
        z_atual += velocidade
        
        if z_atual > z_final: 
            z_atual = z_final
        
        nova_pos[2] = z_atual
        sim.setObjectPosition(target, -1, nova_pos)
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
    
def alinharComObjeto(obj_path):
    pos_alvo = sim.getObjectPosition(sim.getObject(obj_path), -1)
    pos_atual = sim.getObjectPosition(target, -1)
    
    nova_pos = [pos_alvo[0], pos_alvo[1], pos_atual[2]]
    sim.setObjectPosition(target, -1, nova_pos)
    time.sleep(0.1)

def pegaBlocoEsteira():
    abrirGarra()
    
    bloco = "/esteiraColeta"
    altura = 0.02
    
    # Alinha o target horizontalmente com o bloco
    alinharComObjeto(bloco)
    
    # z Final
    z_bloco = sim.getObjectPosition(sim.getObject(bloco), -1)[2] + altura
    descerBraco(z_bloco)
    
    #Implementar o Fecha Garra
    fecharGarra()
    time.sleep(2)
    # Sobe de volta à altura anterior (posição de início do movimento)
    altura_inicial = sim.getObjectPosition(sim.getObject("/pontoEspera"), -1)[2]
    subirBraco(altura_inicial)

def moverBraco(x_alvo, y_alvo, velocidade = 0.001, intervalo = 0.0001):
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
    descerBraco(pos_Entrega[2] + 0.02, velocidade=0.001)
    time.sleep(1)
    abrirGarra()
    time.sleep(1)
    
    posEspera = sim.getObjectPosition(sim.getObject("/pontoEspera"), -1)
    subirBraco(posEspera[2])
    moverBraco(posEspera[0], posEspera[1])

# Comunicação com MQTT
# Subscriber
def on_message(client, userdata, msg):
    global espera_bloco, destino_livre, finalizado
    print(f"Mensagem recebida: {msg.topic} -> {msg.payload.decode()}")

    if msg.topic == "/bloco/disponivel":
        espera_bloco = True
        print("[MQTT] Bloco disponível, iniciando coleta.")
    elif msg.topic == "/entregador/pontoRecebimento":
        destino_livre = True
        print("[MQTT] Destino disponível, iniciando entrega.")
    elif msg.topic == "/colaboracao/fim":
        finalizado = True
        print("[MQTT] Colaboração Finalizada.")
    
# Broker
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect("localhost", 1883, 60)    

mqtt_client.subscribe("/bloco/disponivel", qos=1)
mqtt_client.subscribe("/entregador/pontoRecebimento")
mqtt_client.subscribe("/colaboracao/fim")


def mqtt_loop():
    mqtt_client.loop_forever()

# Inicia thread do MQTT
threading.Thread(target=mqtt_loop, daemon=True).start()

# Executa
conectar()
obterHandles()
ikGarra()


while True:
    
    if espera_bloco and not segurando_bloco:
        pegaBlocoEsteira()
        print(f"Bloco pego!")
        espera_bloco = False
        segurando_bloco = True

    if destino_livre and segurando_bloco:
        entregaBloco()
        print(f"Bloco entregue!")
        mqtt_client.publish("/entregador/encomendaDisponibilizada", payload="true")
        destino_livre = False
        segurando_bloco = False
        
    if finalizado:
        break

    time.sleep(0.1)