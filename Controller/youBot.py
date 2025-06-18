# Biblioteca para o Coppelia
import sim
# Bibliotecas uteis
import numpy as np
import matplotlib.pyplot as plt
import time

# Variaveis globais
ip = '127.0.0.1'
port = 19999
clientId = None

# Nome dos Objetos
robotName = 'youBot'
motorFrontDir = 'rollingJoint_rr'
motorFrontEsq = 'rollingJoint_rl'
motorTrasDir = 'rollingJoint_fr'
motorTrasEsq = 'rollingJoint_fl'

# Handle do Robot
handleRobot = None
handleMotorFrontDir = None
handleMotorFrontEsq = None
handleMotorTrasDir = None
handleMotorTrasEsq = None
areaEntrega = None
areaRecebimento = None

def conectar():
    global clientId
    
    # Fechar conecões
    sim.simxFinish(-1)
    
    # Conectar com o Coppelia
    clientId = sim.simxStart(ip, port, True, True, 5000, 5)
    
    if clientId != 1:
        print("Conexão via API realizada com sucesso")

def obterHandles():
    global handleRobot, handleMotorFrontDir, handleMotorFrontEsq, handleMotorTrasDir, handleMotorTrasEsq, areaEntrega, areaRecebimento
    
    # Handle do Robot
    returnCode, handleRobot = sim.simxGetObjectHandle(clientId, robotName, sim.simx_opmode_oneshot_wait)
    
    # Handles dos motores frontais
    returnCode, handleMotorFrontDir = sim.simxGetObjectHandle(clientId, motorFrontDir, sim.simx_opmode_oneshot_wait)
    returnCode, handleMotorFrontEsq = sim.simxGetObjectHandle(clientId, motorFrontEsq, sim.simx_opmode_oneshot_wait)
    
    # Handles dos motores traseiros
    returnCode, handleMotorTrasDir = sim.simxGetObjectHandle(clientId, motorTrasDir, sim.simx_opmode_oneshot_wait)
    returnCode, handleMotorTrasEsq = sim.simxGetObjectHandle(clientId, motorTrasEsq, sim.simx_opmode_oneshot_wait)
    
    # Areas de recebimento e entrega
    returnCode, areaRecebimento = sim.simxGetObjectHandle(clientId, "recebe_caixa", sim.simx_opmode_oneshot_wait)
    returnCode, areaEntrega = sim.simxGetObjectHandle(clientId, "entrega_caixa", sim.simx_opmode_oneshot_wait)
    
    if returnCode == sim.simx_return_ok:
        print('Handles obtidos com sucesso.')
    else:
        print('Erro ao obter um ou mais handles.')
    
def setVelocidade(vel):
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontDir, vel, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontEsq, vel, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasDir, vel, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasEsq, vel, sim.simx_opmode_streaming + 5)

def virarDireita(velPosit, velNeg):
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontDir, velPosit, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontEsq, velNeg, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasDir, velPosit, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasEsq, velNeg, sim.simx_opmode_streaming + 5)
    
def virarEsquerda(velPosit, velNeg):
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontDir, velNeg, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorFrontEsq, velPosit, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasDir, velNeg, sim.simx_opmode_streaming + 5)
    sim.simxSetJointTargetVelocity(clientId, handleMotorTrasEsq, velPosit, sim.simx_opmode_streaming + 5)

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
        rc_pos, posRobot_xyz = sim.simxGetObjectPosition(clientId, handleRobot, -1, sim.simx_opmode_oneshot_wait)
        
        if rc_pos != sim.simx_return_ok:
            print("Erro ao obter posição do robô.")
            break
        posRobot = np.array([posRobot_xyz[0], posRobot_xyz[1]])

        # Obter orientação
        rc_ori, orientacao = sim.simxGetObjectOrientation(clientId, handleRobot, -1, sim.simx_opmode_oneshot_wait)
        
        if rc_ori != sim.simx_return_ok:
            print("Erro ao obter orientação do robô.")
            break
        orientacao = np.array(orientacao)
        orientacao = orientacao * 180/np.pi
        orientacao = np.around(orientacao, decimals=0)
        print(orientacao)
        
        ang, rad = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
        print(f"Angulo: {ang}\nRadianos: {rad}")

        # Diferenca entre pontos
        diferenca = alvo - posRobot
        # Norma (magnitude)
        distancia = np.linalg.norm(diferenca)

        if distancia < 0.1:
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
        rc_ori, orientacao = sim.simxGetObjectOrientation(clientId, handleRobot, -1, sim.simx_opmode_oneshot_wait)
        
        if rc_ori != sim.simx_return_ok:
            print("Erro ao obter orientação do robô.")
            break
        
        orientacao = np.array(orientacao)
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
    rc_entrega, entregaPos = sim.simxGetObjectPosition(clientId, areaEntrega, -1, sim.simx_opmode_oneshot_wait)
    if rc_entrega == sim.simx_return_ok:
        entregaPos_xy = np.array([entregaPos[0], entregaPos[1]])
        moverRobo(entregaPos_xy)
        orientarRobo(anguloAlvo=90)
    else:
        print("Erro ao obter posição da área de entrega.")
    
def recebeCaixa():
    rc_recebe, recebePos = sim.simxGetObjectPosition(clientId, areaRecebimento, -1, sim.simx_opmode_oneshot_wait)
    if rc_recebe == sim.simx_return_ok:
        recebePos_xy = np.array([recebePos[0], recebePos[1]])
        moverRobo(recebePos_xy)
        orientarRobo(anguloAlvo=90)
    else:
        print("Erro ao obter posição da área de recebimento.")
       
conectar()
obterHandles()
recebeCaixa()