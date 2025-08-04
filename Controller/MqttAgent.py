import paho.mqtt.client as mqtt
from types import MappingProxyType
import json

class MqttAgent:
    # multiagent.ddns.net
    # ip: ip do edison
    def __init__(self, topicos_mqtt: list, broker: str = "localhost", port: int = 1883):
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        
        # Definindo autenticação
        self.client.username_pw_set(username="kelton", password="Projeto2025")
        
        # Estados internos
        self._espera_bloco = False
        self._destino_livre = False
        self._finalizado = False
        self._iniciar_entrega = False
        self._iniciar_coleta = True
        self._cubo = None
        
        # Mapeamento de tópicos dos agentes
        self.topic_map = MappingProxyType({
            "/entregador/coletaDisponivel": self.tratar_coleta_disponivel,
            "/bloco/disponivel": self.tratar_coleta_disponivel,
            "/entregador/pontoRecebimento": self.tratar_ponto_recebimento,
            "/entregador/encomendaDisponibilizada": self.tratar_encomenda_disponibilizada,
            "/entregador/encomendaColetada": self.tratar_encomenda_coletada,
            "/colaboracao/fim": self.tratar_fim_colaboracao,
        })
        
        self.client.connect(broker, port, 60)
        
        self.subscribe(topicos_mqtt)
        self.iniciar()
    
    def subscribe(self, topics: list[tuple | str]):
        for topico in topics:
            if not isinstance(topico, tuple):
                self.client.subscribe(topico)                
            else:
                self.client.subscribe(topico[0], qos=topico[1])
    
    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        payload = json.loads(payload)
        print(f"[MQTT] {msg.topic} -> {payload}")
        
        handler = self.topic_map.get(msg.topic)
        if handler:
            handler(payload)
        else:
            print(f"[MQTT] Aviso: tópico não tratado: {msg.topic}")
    
    def iniciar(self):
        self.client.loop_start()
        
    def publicar(self, canal: str, msg: dict, qos: int = 0):
        msg = json.dumps(msg)
        self.client.publish(canal, payload=msg, qos=qos)
    
    def desconectar(self):
        print("[MQTT] Desconectando do broker...")
        self.client.loop_stop()
        self.client.disconnect()
        print("[MQTT] Desconectado.")
        
    # Tratamentos
    def tratar_coleta_disponivel(self, payload):
        self.espera_bloco = True
        print("[MQTT] Bloco disponível, iniciando coleta.")
        
        if "cubo" in payload:
            self._cubo = payload["cubo"]

    def tratar_ponto_recebimento(self, payload):
        self.destino_livre = True
        print("[MQTT] Destino disponível, iniciando entrega.")

    def tratar_encomenda_disponibilizada(self, payload):
        self.iniciar_entrega = True
        print("[MQTT] Recebido: Bloco recebido. Indo entregar.")

    def tratar_encomenda_coletada(self, payload):
        self.iniciar_coleta = True
        print("[MQTT] Recebido: Bloco entregue. Indo coletar.")

    def tratar_fim_colaboracao(self, payload):
        self.finalizado = True
        print("[MQTT] Colaboração Finalizada.")
    
    # Fim dos Tratamentos
    
    @property
    def espera_bloco(self):
        return self._espera_bloco

    @espera_bloco.setter
    def espera_bloco(self, valor: bool):
        self._espera_bloco = valor

    @property
    def destino_livre(self):
        return self._destino_livre

    @destino_livre.setter
    def destino_livre(self, valor: bool):
        self._destino_livre = valor

    @property
    def finalizado(self):
        return self._finalizado

    @finalizado.setter
    def finalizado(self, valor: bool):
        self._finalizado = valor

    @property
    def iniciar_entrega(self):
        return self._iniciar_entrega

    @iniciar_entrega.setter
    def iniciar_entrega(self, valor: bool):
        self._iniciar_entrega = valor

    @property
    def iniciar_coleta(self):
        return self._iniciar_coleta

    @iniciar_coleta.setter
    def iniciar_coleta(self, valor: bool):
        self._iniciar_coleta = valor
        
    @property
    def cubo(self):
        return self._cubo