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

def posicaoObjeto(objeto):
    _, posicao = sim.simxGetObjectPosition(client_id, objeto, -1, sim.simx_opmode_blocking)
    return posicao

def orientacaoObjeto(objeto):
    _, orientation = sim.simxGetObjectOrientation(client_id, objeto, -1, sim.simx_opmode_blocking)
    return orientation

def ir_ate_caixa():
    limite_distancia = 0.5
    velocidade_linear = -2.0  # Velocidade para frente (ajuste conforme necessário)
    velocidade_angular = 0.5  # Velocidade de giro (ajuste conforme necessário)
    tolerancia_angulo = math.radians(5)  # 5 graus de tolerância para o alinhamento

    while True:
        posRobot = posicaoObjeto(robot)
        posCaixa = posicaoObjeto(caixa)
        orientRobot = orientacaoObjeto(robot) # Orientação do robô em radianos (pitch, yaw, roll)

        print(f"Posicao Robo: {posRobot}\nPosicao Caixa: {posCaixa}")
        print(f"Orientacao Robo: {orientRobot[2]} radianos") # Usamos a orientação no eixo Z (yaw)

        # Diferença entre os pontos
        dx = posCaixa[0] - posRobot[0]
        dy = posCaixa[1] - posRobot[1]
        distancia = math.hypot(dx, dy)
        print(f"dx: {dx}\ndy: {dy}\ndistancia: {distancia}")

        # Calcular o ângulo alvo para a caixa
        angulo_alvo = math.atan2(dy, dx)
        
        # Obter o ângulo atual do robô (yaw)
        angulo_robo = orientRobot[2] # Considerando que a rotação em Z é o yaw

        # Calcular a diferença angular
        # Normalizar o ângulo para estar entre -pi e pi
        diferenca_angulo = angulo_alvo - angulo_robo
        if diferenca_angulo > math.pi:
            diferenca_angulo -= 2 * math.pi
        elif diferenca_angulo < -math.pi:
            diferenca_angulo += 2 * math.pi

        print(f"Angulo alvo: {math.degrees(angulo_alvo):.2f} graus")
        print(f"Angulo robo: {math.degrees(angulo_robo):.2f} graus")
        print(f"Diferença de angulo: {math.degrees(diferenca_angulo):.2f} graus")

        # Se o robô estiver perto, pare
        if distancia < limite_distancia:
            stopRobot()
            print("Robô perto do alvo. Parando.")
            break
        
        # Se o robô não estiver alinhado, gire
        if abs(diferenca_angulo) > tolerancia_angulo:
            if diferenca_angulo > 0: # Precisa girar para a esquerda (anti-horário)
                virar4rodas(-velocidade_angular, velocidade_angular) # Roda esquerda pra trás, roda direita pra frente
                print("Girando para a esquerda...")
            else: # Precisa girar para a direita (horário)
                virar4rodas(velocidade_angular, -velocidade_angular) # Roda esquerda pra frente, roda direita pra trás
                print("Girando para a direita...")
        else: # Se o robô estiver alinhado, vá para frente
            setVelocidade(velocidade_linear)
            print("Indo para frente...")
            
        time.sleep(0.1)

    
client_id = conectar()
if client_id != -1:
    obter_handles(client_id)
    ir_ate_caixa()
    desconectar()
else:
    print("Não foi possível iniciar a simulação. Verifique sua conexão com o CoppeliaSim.")