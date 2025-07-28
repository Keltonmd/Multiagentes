# 🤖 Projeto Multiagentes com CoppeliaSim e MQTT

**Aluno:** Kelton Martins Dias

**Orientador:** Prof. Felipe Mota

**Instituição:** Instituto Federal do Norte de Minas Gerais – Campus Januária

## 🧠 Visão Geral

Este projeto simula um sistema multiagente robótico utilizando o simulador **CoppeliaSim** e comunicação por meio do protocolo **MQTT**.
A proposta envolve múltiplos robôs autônomos que colaboram para manipular blocos em um ambiente simulado.

## ⚙️ Tecnologias Utilizadas

* **CoppeliaSim** (simulador de robótica)
* **MQTT para comunicação entre agentes**
* **paho-mqtt** (cliente MQTT em Python)
* **Mosquitto** (servidor MQTT broker)
* **Python 3.10+**

## 🤖 Agentes Envolvidos

| Agente     | Função                                                       |
| ---------- | ------------------------------------------------------------ |
| **Franka** | Coleta o bloco da esteira e entrega ao youBot.               |
| **youBot** | Recebe o bloco do Franka e leva até o robô UR10.             |
| **UR10**   | Pega o bloco do youBot e armazena cuidadosamente na estante. |
| **Sensor** | Detecta a presença de blocos na esteira e aciona o processo. |

## 🎯 Missão

Automatizar o transporte e armazenamento de blocos por meio de agentes robóticos, utilizando controle distribuído e comunicação assíncrona por **MQTT**, com coordenação entre diferentes robôs no ambiente simulado do CoppeliaSim.

---

## 🔁 Fluxo de Execução

1. O **sensor** detecta um novo bloco na esteira e envia uma mensagem MQTT ao **Franka**.
2. O **Franka** coleta o bloco da esteira.
3. O bloco é transferido para o **youBot**.
4. O **youBot** transporta o bloco até o **UR10**.
5. O **UR10** pega o bloco e o armazena em uma **estante**.
6. O ciclo reinicia quando o próximo bloco é detectado.

---

## 📁 Estrutura do Projeto

```
Multiagentes/
├── Controller/
│   ├── main.py
│   ├── franka.py
│   ├── youBot.py
│   ├── ur10.py
│   ├── sensorEsteira.py
│   ├── CoppeliaBracoAgent.py
│   ├── CoppeliaMobileAgent.py
│   └── MqttAgent.py
├── Coppelia/
│   └── Multiagentes.ttt
└── README.md
```

---

## ▶️ Como Executar

### 1. Clone o Repositório

```bash
git clone https://github.com/Keltonmd/Multiagentes.git
cd Multiagentes/Controller
```

### 2. Crie o Ambiente Virtual

```bash
python -m venv venv
source venv/bin/activate     # Linux/macOS
venv\Scripts\activate.bat    # Windows
```

### 3. Instale as Dependências

```bash
pip install -r requirements.txt
```

### 4. Instale e Inicie o Mosquitto Broker

#### Linux (Ubuntu)

```bash
sudo apt update
sudo apt install mosquitto
sudo systemctl enable mosquitto
sudo systemctl start mosquitto
```

#### Windows

* Baixe o Mosquitto: [https://mosquitto.org/download/](https://mosquitto.org/download/)
* Instale e execute como serviço ou inicie com `mosquitto.exe`

### 5. Inicie o CoppeliaSim

* Abra o projeto `Multiagentes.ttt` que está na pasta `Coppelia/`.

### 6. Execute o Script Principal

```bash
python main.py
```

O `main.py` será responsável por inicializar e orquestrar os scripts de todos os agentes.

---

## ✅ Requisitos

* Python 3.10+
* CoppeliaSim instalado
* Broker MQTT (Mosquitto)

---

## 📚 Possibilidades de Expansão

* Monitoramento em tempo real com painel de controle em Flask ou Dash
* Integração com sensores físicos reais via MQTT
* Visualização de métricas como tempo de coleta, transporte e armazenamento
* Módulo de aprendizagem autônoma para otimização de rotas

---

## 📸 Ilustrações (futuramente)

*Imagens do cenário em execução no CoppeliaSim, com destaque para os robôs e posições dos blocos.*

---