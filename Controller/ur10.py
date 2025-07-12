# Biblioteca para o Coppelia (versão nova com ZMQ API)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import time
import threading
import paho.mqtt.client as mqtt

# Variveis globais de conexão
client = None
sim = None
simIK = None
mqtt_client = None

# Handles
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
posDisponivel = []

# Variaveis para colaboração Memoria do Agente
espera_bloco = False
segurando_bloco = False
finalizado = False


def conectar():
    global client, sim, simIK
    client = RemoteAPIClient()
    sim = client.require('sim')
    simIK = client.require('simIK')
    
def obterHandles():
    global cubo, target, posDisponivel
    global handleActive1, handleActive2, estante
    
    # Obter o target
    target = sim.getObject("/UR10/target")
    
    # Garra
    handleActive1 = sim.getObject("/UR10/ROBOTIQ85/active1")
    handleActive2 = sim.getObject("/UR10/ROBOTIQ85/active2")
    
    # Estante e Cubo
    estante = sim.getObject("/rack/")
    
    for i in range(10):
        pos = sim.getObjectPosition(sim.getObject('/rack/pos', {'index': i}), -1)
        posDisponivel.append({'pos': pos, 'livre': True})
        
    cubo = sim.getObject("/youBot/cuboPos")

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

# IK da Garra
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

# Funcoes da Garra    
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

# Funcoes do Braco 
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

def moverVertical(y_alvo, velocidade=0.0005, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    y_atual = nova_pos[1]
    
    while y_atual != y_alvo:
        if y_atual < y_alvo:
            y_atual += velocidade
            if y_atual > y_alvo:
                y_atual = y_alvo
        else:
            y_atual -= velocidade
            if y_atual < y_alvo:
                y_atual = y_alvo
        
        nova_pos[1] = y_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def moverHorizontal(x_alvo, velocidade=0.0005, intervalo=0.0001):
    nova_pos = sim.getObjectPosition(target, -1)
    x_atual = nova_pos[0]
    
    while x_atual != x_alvo:
        if x_atual < x_alvo:
            x_atual += velocidade
            if x_atual > x_alvo:
                x_atual = x_alvo
        else:
            x_atual -= velocidade
            if x_atual < x_alvo:
                x_atual = x_alvo
        
        nova_pos[0] = x_atual
        sim.setObjectPosition(target, -1, nova_pos)
        time.sleep(intervalo)

def moverXYZ(x_alvo, y_alvo, z_alvo, velocidade=0.001, intervalo=0.001):
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

def rotacionarXY(ang_final, espera, velocidade = 0.1, intervalo = 0.0000001):
    nova_ori = sim.getObjectOrientation(target, -1)
    orientacao = np.around(np.degrees(np.array(nova_ori)), decimals=0)
    ang_atual, _ = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
    
    nova_pos = sim.getObjectPosition(target, -1)
    posEspera = sim.getObjectPosition(espera, -1)
    x_atual = nova_pos[0]
    y_atual = nova_pos[1]
    x_alvo = posEspera[0]
    y_alvo = posEspera[1]

    while ang_atual != ang_final or x_atual != x_alvo or y_atual != y_alvo:
        if x_atual != x_alvo:
            if x_atual < x_alvo:
                x_atual += velocidade * 0.005
                if x_atual > x_alvo:
                    x_atual = x_alvo
            else:
                x_atual -= velocidade * 0.005
                if x_atual < x_alvo:
                    x_atual = x_alvo
                    
        if y_atual != y_alvo:
            if y_atual < y_alvo:
                y_atual += velocidade * 0.005
                if y_atual > y_alvo:
                    y_atual = y_alvo
            else:
                y_atual -= velocidade * 0.005
                if y_atual < y_alvo:
                    y_atual = y_alvo
                    
        if ang_atual != ang_final:
            if ang_atual > ang_final:
                ang_atual -= velocidade
                if ang_atual < ang_final: 
                    ang_atual = ang_final
                
            if ang_atual < ang_final:
                ang_atual += velocidade
                if ang_atual > ang_final: 
                    ang_atual = ang_final
        
        nova_ori[2] = np.radians(ang_atual)
        sim.setObjectOrientation(target, -1, nova_ori)
        
        nova_pos[0] = x_atual
        nova_pos[1] = y_atual
        sim.setObjectPosition(target, -1, nova_pos)
        
        time.sleep(intervalo)
    
# Acoes
def pegarBloco():
    espera = sim.getObject("/UR10/posEspera")
    rotacionarXY(0, espera)
    
    time.sleep(1.5)
    abrirGarra()
    
    # Pega a posição do cubo que está no youBot
    pos_cubo = sim.getObjectPosition(cubo, -1)
    
    if pos_cubo[1] > 0:
        pos_cubo[1] -= 0.025
    else:
        pos_cubo[1] += 0.025
        
    moverXYZ(pos_cubo[0], pos_cubo[1], pos_cubo[2])
    
    time.sleep(1)
    fecharGarra()
    time.sleep(2)
    
    posEspera = sim.getObjectPosition(sim.getObject("/UR10/posEspera"), -1)
    subirBraco(posEspera[2])
    
    moverXYZ(posEspera[0], posEspera[1], posEspera[2])

def guardarBloco():
    time.sleep(3)
    espera = sim.getObject("/UR10/posEsperaAtras")
    rotacionarXY(180, espera)
    
    time.sleep(1)
    
    pos = []
    for posicao in posDisponivel:
        if posicao["livre"]:
            posicao["livre"] = False
            pos = posicao["pos"]
            break
            
    if not pos:
        return
    
    descerBraco(pos[2] + 0.01)
    moverHorizontal(pos[0])
    moverVertical(pos[1] + 0.01)
    
    time.sleep(1)
    abrirGarra()
    time.sleep(1)
    
    posEspera = sim.getObjectPosition(sim.getObject("/UR10/posEsperaAtras"), -1)
    
    moverVertical(posEspera[1])
    moverHorizontal(posEspera[0])
    subirBraco(posEspera[2])
    
    time.sleep(1.5)
    
    espera = sim.getObject("/UR10/posEspera")
    rotacionarXY(0, espera)

# Comunicação com MQTT
# Subscriber
def on_message(client, userdata, msg):
    global espera_bloco, destino_livre, finalizado
    print(f"Mensagem recebida: {msg.topic} -> {msg.payload.decode()}")

    if msg.topic == "/entregador/coletaDisponivel":
        espera_bloco = True
        print("[MQTT] Bloco disponível, iniciando coleta.")
    elif msg.topic == "/colaboracao/fim":
        finalizado = True
        print("[MQTT] Colaboração Finalizada.")

# Broker
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_message
mqtt_client.connect("localhost", 1883, 60)    

mqtt_client.subscribe("/entregador/coletaDisponivel")
mqtt_client.subscribe("/colaboracao/fim")

def mqtt_loop():
    mqtt_client.loop_forever()

# Inicia thread do MQTT
threading.Thread(target=mqtt_loop, daemon=True).start()

conectar()
obterHandles()
ikGarra()


while True:
    if espera_bloco and not segurando_bloco:
        pegarBloco()
        print(f"Bloco pego!")
        mqtt_client.publish("/entregador/encomendaColetada", payload="true")
        espera_bloco = False
        segurando_bloco = True
        
    if segurando_bloco:
        guardarBloco()
        print(f"Bloco guardado!")
        segurando_bloco = False
    
    if finalizado:
        break
        