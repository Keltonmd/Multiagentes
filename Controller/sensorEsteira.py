from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import paho.mqtt.client as mqtt

client = None
sim = None
mqtt_client = None


# Variaveis Globais
sensorHandle = None

ultimoEstado = False

def conectar():
    global client, sim, mqtt_client
    client = RemoteAPIClient()
    sim = client.require('sim')
    
    # Broker
    mqtt_client = mqtt.Client()
    mqtt_client.connect("localhost", 1883, 60)
    
def obterHandle():
    global sensorHandle
    sensorHandle = sim.getObject("/gera_caixa/proximitySensor")
  
def analisar():
    global ultimoEstado
    print("Iniciando")
    
    while True:
        result, _, _, _, _ = sim.readProximitySensor(sensorHandle)
        
        if result > 0 and not ultimoEstado:
            print("[SENSOR] Bloco detectado. Publicando...")
            mqtt_client.publish("/bloco/disponivel", payload="true")
            ultimoEstado = True
        elif result == 0:
            ultimoEstado = False
                  
conectar()
obterHandle()  
analisar()