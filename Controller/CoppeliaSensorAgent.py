from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time

class CoppeliaSensorAgent:
    def __init__(self, caminho):
        self.caminho = caminho
        
        # variaveis padrão
        self.client = None
        self.sim = None
        
        self.baseSensor = None
        
        self.inicializador()
    
    def inicializador(self):    
        self.conectar()
        self.obterBase()
    
    def conectar(self):
        self.client = RemoteAPIClient()
        self.sim = self.client.require('sim')
        
    def obterBase(self):
        self.baseSensor = self.sim.getObject(self.caminho)
        
    def leitura(self):
        result, _, _, _, _ = self.sim.readProximitySensor(self.baseSensor)
        return bool(result)
    
    def handleObjeto(self, caminho):
        return self.sim.getObject(caminho)