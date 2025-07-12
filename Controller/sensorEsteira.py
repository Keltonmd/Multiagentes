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
    mqtt_client.loop_start()
    
def obterHandle():
    global sensorHandle
    sensorHandle = sim.getObject("/gera_caixa/proximitySensor")
  
def analisar():
    global ultimoEstado
    print("Iniciando")
    
    cont = 0
    while True:
        result, _, _, _, _ = sim.readProximitySensor(sensorHandle)
        
        if result > 0 and not ultimoEstado:
            print("[SENSOR] Bloco detectado. Publicando...")
            mqtt_client.publish("/bloco/disponivel", payload="true", qos=1)
            ultimoEstado = True
            if cont >= 9:
                mqtt_client.publish("/colaboracao/fim", payload="true", qos=1)
                break
            cont += 1
        elif result == 0:
            ultimoEstado = False
        
        time.sleep(0.1)
        
                  
conectar()
obterHandle()  
analisar()