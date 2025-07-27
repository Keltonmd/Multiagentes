from CoppeliaMobileAgent import CoppeliaMobileAgent
from MqttAgent import MqttAgent
import time
import numpy as np

agent = CoppeliaMobileAgent("/youBot", ['/rollingJoint_rr', '/rollingJoint_rl', '/rollingJoint_fr', '/rollingJoint_fl'])

areaEntrega = agent.getPos('/entrega_caixa')
areaRecebimento = agent.getPos('/recebe_caixa')

topicos = ["/entregador/encomendaDisponibilizada", "/entregador/encomendaColetada", "/colaboracao/fim"]
client = MqttAgent(topicos)

def entregar():
    agent.moverRobo(np.array([areaEntrega[0], areaEntrega[1]]))
    agent.oritentarRobo()

def receber():
    agent.moverRobo(np.array([areaRecebimento[0], areaRecebimento[1]]))
    agent.oritentarRobo()
    
while True:
    if client.get_iniciar_entrega():
        entregar()
        print(f"Ponto de Recebimento")
        client.publicar("/entregador/coletaDisponivel")
        client.set_iniciar_entrega(False)

    if client.get_iniciar_coleta():
        receber()
        print(f"Ponto de Recebimento")
        client.publicar("/entregador/pontoRecebimento")
        client.set_iniciar_coleta(False)

    if client.get_finalizado():
        break
