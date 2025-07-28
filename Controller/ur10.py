from CoppeliaBracoAgent import CoppeliaBracoAgent
from MqttAgent import MqttAgent
import time

agent = CoppeliaBracoAgent("/UR10")
posDisponivel = agent.getPosicoesRack("/rack/pos", 10)

topicos = ["/entregador/coletaDisponivel", "/colaboracao/fim"]
client = MqttAgent(topicos)
segurando_bloco = False

def pegarBloco():
    espera = agent.getObjeto("/UR10/posEspera")
    agent.rotacionar_para_posicao_xyz(0,espera)
    time.sleep(1.5)
    agent.abrirGarra()
    
    pos_cubo = agent.getPos("/youBot/cuboPos")
    
    if pos_cubo[1] > 0:
        pos_cubo[1] -= 0.025
    else:
        pos_cubo[1] += 0.025
        
    agent.mover_para_posicao_xyz([pos_cubo[0], pos_cubo[1], pos_cubo[2]])
    time.sleep(1)
    agent.fecharGarra()
    time.sleep(2)
    
    posEspera = agent.getPos("/UR10/posEspera")
    agent.subirBraco(posEspera[2])
    
    agent.mover_para_posicao_xyz([posEspera[0], posEspera[1], posEspera[2]])

def guardarBloco():
    time.sleep(3)
    espera = agent.getObjeto("/UR10/posEsperaAtras")
    agent.rotacionar_para_posicao_xyz(180, espera)
    
    time.sleep(1)
    pos = []
    for posicao in posDisponivel:
        if posicao["livre"]:
            posicao["livre"] = False
            pos = posicao["pos"]
            break
            
    if not pos:
        return
    
    agent.descerBraco(pos[2] + 0.01)
    # Mover na horinzontal
    agent.mover_para_posicao_xyz([pos[0], None, None])
    # Mover na vertical
    agent.mover_para_posicao_xyz([None, pos[1], None])
    
    
    time.sleep(1)
    agent.abrirGarra()
    time.sleep(1)
    
    posEspera = agent.getPos("/UR10/posEsperaAtras")
    
    # Mover na vertical
    agent.mover_para_posicao_xyz([None, posEspera[1], None])
    
    # Mover na horinzontal
    agent.mover_para_posicao_xyz([posEspera[0], None, None])
    
    agent.subirBraco(posEspera[2] + 0.01)
    
    time.sleep(1.5)
    
    espera = agent.getObjeto("/UR10/posEspera")
    agent.rotacionar_para_posicao_xyz(0, espera)

while True:
    if client.espera_bloco and not segurando_bloco:
        pegarBloco()
        print(f"Bloco pego!")
        client.publicar("/entregador/encomendaColetada")
        client.espera_bloco = False
        segurando_bloco = True
        
    if segurando_bloco:
        guardarBloco()
        print(f"Bloco guardado!")
        segurando_bloco = False
        
    if client.finalizado:
        client.desconectar()
        break