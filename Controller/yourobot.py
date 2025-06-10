import sim
import time
import math

# Variáveis globais
robotNome = "youBot"
motorRightTR = 'rollingJoint_rr'
motorLeftTR = 'rollingJoint_rl'
motorRightFR = 'rollingJoint_fr'
motorLeftFR = 'rollingJoint_fl'
port = 19997
ip = '127.0.0.1'

# Handles globais
client_id = None
motorTrDr = None
motorTrEsq = None
motorFrDr = None
motorFrEsq = None
robot = None
caixa = None


def conectar():
    sim.simxFinish(-1)  # Fecha todas conexões anteriores
    client_id = sim.simxStart(ip, port, True, True, 2000, 5)
    if client_id != -1:
        print(f"[INFO] Conectado ao CoppeliaSim com client_id: {client_id}")
    else:
        print("[ERRO] Falha ao conectar ao CoppeliaSim.")
    return client_id
 
def desconectar():
    sim.simxFinish(-1)
    
def obter_handles(client_id):
    # Usando as variaveis goblais
    global motorTrDr, motorTrEsq, robot, caixa, motorFrDr, motorFrEsq
    
    # Obtendo os Handles do Robot
    
    # Motores Traseiro
    _, motorTrDr = sim.simxGetObjectHandle(client_id, motorRightTR, sim.simx_opmode_blocking)
    _, motorTrEsq = sim.simxGetObjectHandle(client_id, motorLeftTR, sim.simx_opmode_blocking)
    
    # Motores Frontais
    _, motorFrDr = sim.simxGetObjectHandle(client_id, motorRightFR, sim.simx_opmode_blocking)
    _, motorFrEsq = sim.simxGetObjectHandle(client_id, motorLeftFR, sim.simx_opmode_blocking)
    
    # Obtendo o robo e a caixa
    _, robot = sim.simxGetObjectHandle(client_id, robotNome, sim.simx_opmode_blocking)
    _, caixa = sim.simxGetObjectHandle(client_id, 'Caixa', sim.simx_opmode_blocking)
    
    # Mostrando o que foi obtido
    print(f"[INFO] Handles obtidos: {motorRightTR} ({motorTrDr}), {motorLeftTR} ({motorTrEsq})")

def setVelocidade(vel):
    # Velocidade do motor Traseiro
    # Motor esquerdo
    sim.simxSetJointTargetVelocity(client_id, motorTrEsq, vel, sim.simx_opmode_blocking)
    # Motor Direito
    sim.simxSetJointTargetVelocity(client_id, motorTrDr, vel, sim.simx_opmode_blocking)
    
    # Velocidade do motor Frontal
    # Motor esquerdo
    sim.simxSetJointTargetVelocity(client_id, motorFrEsq, vel, sim.simx_opmode_blocking)
    
    # Motor Direito
    sim.simxSetJointTargetVelocity(client_id, motorFrDr, vel, sim.simx_opmode_blocking)
    
def virar4rodas(velEsq, velDir):
    # Velocidade do motor Traseiro
    sim.simxSetJointTargetVelocity(client_id, motorTrEsq, velEsq, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorTrDr, velDir, sim.simx_opmode_blocking)
    
    # Velocidade do motor Frontal
    sim.simxSetJointTargetVelocity(client_id, motorFrEsq, velEsq, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorFrDr, velDir, sim.simx_opmode_blocking)

def virarRodasTr(velEsq, velDir):
    # Velocidade do motor Traseiro
    sim.simxSetJointTargetVelocity(client_id, motorTrEsq, velEsq, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorTrDr, velDir, sim.simx_opmode_blocking)

def virarRodasFr(velEsq, velDir):
    # Velocidade do motor Frontal
    sim.simxSetJointTargetVelocity(client_id, motorFrEsq, velEsq, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorFrDr, velDir, sim.simx_opmode_blocking)
    
def stopRobot():
    # Parar o motor da roda Esquerda
    sim.simxSetJointTargetVelocity(client_id, motorTrEsq, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorFrEsq, 0, sim.simx_opmode_blocking)
    
    # Parar o motor da roda Direita
    sim.simxSetJointTargetVelocity(client_id, motorTrDr, 0, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(client_id, motorFrDr, 0, sim.simx_opmode_blocking)

def iniciarPosicao():
    _, posicao = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_streaming)
    _, posicao = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_streaming)

def posicaoRobot():
    _, posicao = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_buffer)
    return posicao

def posicaoCaixa():
    _, posicao = sim.simxGetObjectPosition(client_id, robot, -1, sim.simx_opmode_buffer)
    return posicao

#def ir_ate_caixa():
#    while True:
#       a

client_id = conectar()
obter_handles(client_id)
iniciarPosicao()
posRobot = posicaoRobot()
print(posRobot)
setVelocidade(5)
time.sleep(3)
posRobot = posicaoRobot()
print(posRobot)