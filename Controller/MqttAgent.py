import paho.mqtt.client as mqtt
import threading

class MqttAgent:
    def __init__(self, topicos, broker="localhost", port=1883):
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        
        # Estados internos
        self.espera_bloco = False
        self.destino_livre = False
        self.finalizado = False
        self.iniciar_entrega = False
        self.iniciar_coleta = False
        
        self.client.connect(broker, port, 60)
        
        self.subscribe(topicos)
        self.iniciarThreading()
    
    def subscribe(self, topics):
        """
            topics = [topico ou (topico, qos)]
        """
        for topico in topics:
            if not isinstance(topico, tuple):
                self.client.subscribe(topico)                
            else:
                self.client.subscribe(topico[0], qos=topico[1])
    
    def publicar(self, canal):
        self.client.publish(canal, payload=True)
    
    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        print(f"[MQTT] {msg.topic} -> {payload}")
        
        if msg.topic == "/entregador/coletaDisponivel":
            self.espera_bloco = True
            print("[MQTT] Bloco disponível, iniciando coleta.")

        elif msg.topic == "/bloco/disponivel":
            self.espera_bloco = True
            print("[MQTT] Bloco disponível, iniciando coleta.")

        elif msg.topic == "/entregador/pontoRecebimento":
            self.destino_livre = True
            print("[MQTT] Destino disponível, iniciando entrega.")

        elif msg.topic == "/entregador/encomendaDisponibilizada":
            self.iniciar_entrega = True
            print("[MQTT] Recebido: Bloco recebido. Indo entregar.")

        elif msg.topic == "/entregador/encomendaColetada":
            self.iniciar_coleta = True
            print("[MQTT] Recebido: Bloco entregue. Indo coletar.")

        elif msg.topic == "/colaboracao/fim":
            self.finalizado = True
            print("[MQTT] Colaboração Finalizada.")
    
    def mqtt_loop(self):
        self.client.loop_forever()
    
    def iniciarThreading(self):
        threading.Thread(target=self.mqtt_loop, daemon=True).start()
    
    def get_espera_bloco(self):
        return self.espera_bloco

    def get_destino_livre(self):
        return self.destino_livre

    def get_finalizado(self):
        return self.finalizado

    def get_iniciar_entrega(self):
        return self.iniciar_entrega

    def get_iniciar_coleta(self):
        return self.iniciar_coleta 
    
    def set_espera_bloco(self, valor: bool):
        self.espera_bloco = valor

    def set_destino_livre(self, valor: bool):
        self.destino_livre = valor

    def set_finalizado(self, valor: bool):
        self.finalizado = valor

    def set_iniciar_entrega(self, valor: bool):
        self.iniciar_entrega = valor

    def set_iniciar_coleta(self, valor: bool):
        self.iniciar_coleta = valor