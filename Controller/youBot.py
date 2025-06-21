# Biblioteca para o Coppelia (versão nova com ZMQ API)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import numpy as np
import matplotlib.pyplot as plt
import time

# Variveis globais de conexão
client = None
sim = None
    
# Nome dos Objetos
robotName = '/youBot'
motorFrontDir = '/youBot/rollingJoint_rr'
motorFrontEsq = '/youBot/rollingJoint_rl'
motorTrasDir = '/youBot/rollingJoint_fr'
motorTrasEsq = '/youBot/rollingJoint_fl'

# Handle do Robot
handleRobot = None
handleMotorFrontDir = None
handleMotorFrontEsq = None
handleMotorTrasDir = None
handleMotorTrasEsq = None
areaEntrega = None
areaRecebimento = None

def conectar():
    global client, sim
    client = RemoteAPIClient()
    sim = client.require('sim')

def obterHandles():
    global handleRobot, handleMotorFrontDir, handleMotorFrontEsq, handleMotorTrasDir, handleMotorTrasEsq, areaEntrega, areaRecebimento
    
    handleRobot = sim.getObject(robotName)
    handleMotorFrontDir = sim.getObject(motorFrontDir)
    handleMotorFrontEsq = sim.getObject(motorFrontEsq)
    handleMotorTrasDir = sim.getObject(motorTrasDir)
    handleMotorTrasEsq = sim.getObject(motorTrasEsq)
    areaEntrega = sim.getObject('/entrega_caixa')
    areaRecebimento = sim.getObject('/recebe_caixa')
    
def setVelocidade(vel):
    sim.setJointTargetVelocity(handleMotorFrontDir, vel)
    sim.setJointTargetVelocity(handleMotorFrontEsq, vel)
    sim.setJointTargetVelocity(handleMotorTrasDir, vel)
    sim.setJointTargetVelocity(handleMotorTrasEsq, vel)

def virarDireita(velPosit, velNeg):
    sim.setJointTargetVelocity(handleMotorFrontDir, velPosit)
    sim.setJointTargetVelocity(handleMotorFrontEsq, velNeg)
    sim.setJointTargetVelocity(handleMotorTrasDir, velPosit)
    sim.setJointTargetVelocity(handleMotorTrasEsq, velNeg)

def virarEsquerda(velPosit, velNeg):
    sim.setJointTargetVelocity(handleMotorFrontDir, velNeg)
    sim.setJointTargetVelocity(handleMotorFrontEsq, velPosit)
    sim.setJointTargetVelocity(handleMotorTrasDir, velNeg)
    sim.setJointTargetVelocity(handleMotorTrasEsq, velPosit)

def calcularRotacao(alpha, beta, gamma):
    print(f"Alpha: {alpha}\nBeta: {beta}\nGamma: {gamma}")
    
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
    
    return -1, -1 

def moverRobo(alvo):
    print(f"Iniciando movimento para {alvo}")
    vel_max = 5.0
    vel_min = 3.0
    
    while True:
        # Obter posição do robô
        posRobot_xyz = sim.getObjectPosition(handleRobot, -1)
        posRobot = np.array([posRobot_xyz[0], posRobot_xyz[1]])

        # Obter orientação
        orientacao = np.array(sim.getObjectOrientation(handleRobot, -1))
        orientacao = orientacao * 180/np.pi
        orientacao = np.around(orientacao, decimals=0)
        print(orientacao)
        
        ang, rad = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
        print(f"Angulo: {ang}\nRadianos: {rad}")

        # Diferenca entre pontos
        diferenca = alvo - posRobot
        # Norma (magnitude)
        distancia = np.linalg.norm(diferenca)

        if distancia < 0.09:
            setVelocidade(0)
            print("Destino alcançado!")
            break

        # Orientacao
        angulo_alvo = np.arctan2(diferenca[1], diferenca[0])
        
        # Erro angular com correção de descontinuidade
        erro_angular = angulo_alvo - rad
        erro_angular = (erro_angular + np.pi) % (2 * np.pi) - np.pi
        
        print(f"Angulo alvo: {angulo_alvo * 180/np.pi}\nErro angular: {erro_angular* 180/np.pi}")
        
        # Só anda se estiver bem alinhado (erro pequeno)
        if erro_angular > 0.2:
            setVelocidade(0)
            virarEsquerda(+2, -2)
        elif erro_angular < -0.2:
            setVelocidade(0)
            virarDireita(+2, -2)
        else:
            velocidade = max(vel_min, min(vel_max, distancia * 2))
            setVelocidade(-velocidade)
            
        time.sleep(0.5)

def orientarRobo(anguloAlvo=90):
    angRad = (anguloAlvo * (np.pi / 180))
    while True:
        # Obter orientação
        orientacao = np.array(sim.getObjectOrientation(handleRobot, -1))
        orientacao = orientacao * 180/np.pi
        orientacao = np.around(orientacao, decimals=0)
        print(orientacao)
        
        ang, rad = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
        print(f"Angulo: {ang}\nRadianos: {rad}")
        
        erro_angular = angRad - rad
        erro_angular = (erro_angular + np.pi) % (2 * np.pi) - np.pi
        
        if erro_angular > 0.001:
            setVelocidade(0)
            virarEsquerda(+2, -2)
        elif erro_angular < -0.001:
            setVelocidade(0)
            virarDireita(+2, -2)
        else: 
            setVelocidade(0)
            break

def entregarCaixa():
    entregaPos = sim.getObjectPosition(areaEntrega, -1)
    entregaPos_xy = np.array([entregaPos[0], entregaPos[1]])
    moverRobo(entregaPos_xy)
    orientarRobo(anguloAlvo=90)
    
def recebeCaixa():
    recebePos = sim.getObjectPosition(areaRecebimento, -1)
    recebePos_xy = np.array([recebePos[0], recebePos[1]])
    moverRobo(recebePos_xy)
    orientarRobo(anguloAlvo=90)

       
conectar()
obterHandles()
entregarCaixa()
recebeCaixa()