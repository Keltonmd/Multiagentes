import sim
import numpy as np
import matplotlib.pyplot as plt

port = 19999
ip = '127.0.0.1'
nome_arquivo_pdf = 'Sensor_Hukyo.pdf'

robotName = 'Pionner_p3dx'
juntaEsqName = robotName + '_leftMotor'
juntaDirName = robotName + '_rightMotor'

# Handle para os dados do LASER
laser_distancia_dados = "hokuyo_range_data"
laser_angulo_dados = "hokuyo_angle_data"

client_id = None
robotHandle = None
handleMotorEsq = None
handleMotorDir = None

def conectar():
    global client_id
    sim.simxFinish(-1)
    client_id = sim.simxStart(ip, port, True, True, 5000, 5)

def obterHandle():
    global robotHandle, handleMotorDir, handleMotorEsq
    
    # Obter o handle do Robo
    returnCode, robotHandle = sim.simxGetObjectHandle(client_id, robotName, sim.simx_opmode_oneshot_wait)
    
    # Obter o handle da junta do motor do Robo
    returnCode, handleMotorDir = sim.simxGetObjectHandle(client_id, juntaDirName, sim.simx_opmode_oneshot_wait)
    returnCode, handleMotorEsq = sim.simxGetObjectHandle(client_id, juntaEsqName, sim.simx_opmode_oneshot_wait)
    
    return
    
def lerSensor():
    # === Sincronização de Dados de Sensor (Distância e Ângulo) ===
    # Para garantir que as leituras de distância e ângulo do laser estejam
    # ALINHADAS no tempo e representem o mesmo instante no simulador,
    # usamos uma combinação estratégica dos modos de operação:

    # 1. Inicia o STREAMING dos dados de distância (NÃO BLOQUEANTE):
    #    Solicita que o CoppeliaSim comece a enviar continuamente os dados de distância.
    #    O script não espera, permitindo que o fluxo de dados comece em segundo plano.
    returnCodeDistancia, string_distancia_dados = sim.simxGetStringSignal(client_id, laser_distancia_dados, sim.simx_opmode_streaming)

    # 2. Obtém os dados de ângulo (BLOQUEANTE) e SINCRONIZA:
    #    O script PAUSA e ESPERA (bloqueia) até que os dados de ângulo sejam recebidos.
    #    Isso assegura que, ao obter os ângulos "frescos", os dados de distância
    #    correspondentes no buffer também estarão atualizados e alinhados temporalmente.
    returnCodeAngulo, string_angulo_dados = sim.simxGetStringSignal(client_id, laser_angulo_dados, sim.simx_opmode_blocking)

    # --- Verificação e Desserialização ---
    # Após receber os dados, é necessário verificar o sucesso da leitura e desempacotá-los.
    # Se qualquer returnCode for diferente de 0, a leitura falhou.
    if returnCodeAngulo == 0 and returnCodeDistancia == 0:
        # 'sim.simxUnpackFloats' converte as strings binárias recebidas em listas de números float.
        dados_do_angulo = sim.simxUnpackFloats(string_angulo_dados)
        dados_da_distancia = sim.simxUnpackFloats(string_distancia_dados)

        return dados_da_distancia, dados_do_angulo
    else:
        # Retorna None se a aquisição de um ou ambos os sinais falhou.
        return None

def plotarSensor(dados_da_leitura, max_sensor_distancia = 5):
    fig = plt.figure(figsize=(6,6), dpi=100)
    ax = fig.add_subplot(111, aspect='equal')
    
    for i in range(len(dados_da_leitura)):
        ang, dist = dados_da_leitura[i]
        
        if (max_sensor_distancia - dist) > 0.1:
            x = dist * np.cos(ang)
            y = dist * np.sin(ang)
            c = 'r'
            if ang < 0:    
                c = 'b'
            ax.plot(x, y, 'o', color=c)
            
    ax.plot(x, y, 'k>', markersize=10)
    ax.grid()
    ax.set_xlim([-max_sensor_distancia, max_sensor_distancia])
    ax.set_ylim([-max_sensor_distancia, max_sensor_distancia])
    
    ax.set_xlabel("X (metros)")
    ax.set_ylabel("Y (metros)")
    ax.set_title("Leituras do Sensor de Distância")
    
    plt.savefig(nome_arquivo_pdf, bbox_inches="tight")
    plt.close(fig)

    print(f"Gráfico salvo como: {nome_arquivo_pdf}")

def runCenario():
    conectar()
    
    if client_id == 1:
        print("Falha ao conectar via API com o Coppelia")
        return
    
    print('Conectado via API com o Coppelia')
    
    obterHandle()
    print("Handles Obtidos")
    
    # Em loop até garantir que as leituras serão válidas
    returnCode = 1
    while returnCode != 0:
        returnCode, distancia_dados = sim.simxGetStringSignal(client_id, laser_distancia_dados, sim.simx_opmode_streaming + 10)
        
    # Continuando a Leitura atraves da função ler sensor
    dados_da_distancia, dados_do_angulo = lerSensor()
    dados_da_leitura = np.array([dados_do_angulo, dados_da_distancia]).T
    
    print(dados_da_leitura)
    plotarSensor(dados_da_leitura)
    
    returnCode, pos = sim.simxGetObjectPosition(client_id, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    print(f"Pos: {pos}")
    
runCenario()
    
