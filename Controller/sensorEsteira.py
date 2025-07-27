from CoppeliaSensorAgent import CoppeliaSensorAgent
from MqttAgent import MqttAgent
import time

ultimoEstado = False
agent = CoppeliaSensorAgent("/gera_caixa/proximitySensor")

client = MqttAgent([])

cont = 0
while True:
    detectado = agent.leitura()
    
    if detectado and not ultimoEstado:
        print("[SENSOR] Bloco detectado. Publicando...")
        client.publicar("/bloco/disponivel", qos=1)
        ultimoEstado = True
        if cont >= 9:
            client.publicar("/colaboracao/fim", qos=1)
            print("[SENSOR] Fim da colaboração.")
            break
        cont += 1
    elif not detectado:
        ultimoEstado = False
    
    time.sleep(0.1)