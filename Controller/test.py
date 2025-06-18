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
conversor = None

def conectar():
    global clientId
    
    # Fechar conecões
    sim.simxFinish(-1)
    
    # Conectar com o Coppelia
    clientId = sim.simxStart(ip, port, True, True, 5000, 5)
    
    if clientId != 1:
        print("Conexão via API realizada com sucesso")

def obterHandles():
    global handleRobot, handleMotorFrontDir, handleMotorFrontEsq, handleMotorTrasDir, handleMotorTrasEsq, areaEntrega, areaRecebimento, conversor
    
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
    
conectar()
obterHandles()

rc_pos, posRobot_xyz = sim.simxGetObjectPosition(clientId, handleRobot, -1, sim.simx_opmode_oneshot_wait)
if rc_pos != sim.simx_return_ok:
    print("Erro ao obter posição do robô.")
posRobot = np.array([posRobot_xyz[0], posRobot_xyz[1]])

print(f"Posicçao robo: {posRobot}")

# Obter orientação (yaw)
rc_ori, orientacao = sim.simxGetObjectOrientation(clientId, handleRobot, -1, sim.simx_opmode_oneshot_wait)
if rc_ori != sim.simx_return_ok:
    print("Erro ao obter orientação do robô.")

orientacao = np.array(orientacao)
orientacao = orientacao * 180/np.pi
orientacao = np.around(orientacao, decimals=0)
print(orientacao)

ang, rad = calcularRotacao(orientacao[0], orientacao[1], orientacao[2])
print(f"Angulo: {ang}\nRadianos: {rad}")

rc_entrega, entregaPos = sim.simxGetObjectPosition(clientId, areaEntrega, -1, sim.simx_opmode_oneshot_wait)
if rc_entrega == sim.simx_return_ok:
    entregaPos_xy = np.array([entregaPos[0], entregaPos[1]])
    print(f"Posição de entrega: {entregaPos}")
else:
    print("Erro ao obter posição da área de entrega.")
    
# Cálculo do erro
direcao = entregaPos_xy - posRobot
distancia = np.linalg.norm(direcao)
print(f"Direcao: {direcao}\nDistancia: {distancia}")

angulo_alvo = np.arctan2(direcao[1], direcao[0])

# Erro angular com correção de descontinuidade
erro_angular = angulo_alvo - rad
erro_angular = (erro_angular + np.pi) % (2 * np.pi) - np.pi

print(f"Angulo alvo: {angulo_alvo * 180/np.pi}\nErro angular: {erro_angular* 180/np.pi}")