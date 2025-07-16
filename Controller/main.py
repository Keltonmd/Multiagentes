import subprocess
import time

# Lista dos seus scripts
scripts = [
    "sensorEsteira.py",
    "franka.py",
    "youBot.py",
    "ur10.py"
]

processos = []

# Iniciar todos em subprocessos
for script in scripts:
    print(f"Iniciando {script}...")
    p = subprocess.Popen(["python3", script])
    processos.append(p)
    time.sleep(1) 

# Espera (opcional) até todos terminarem
for p in processos:
    p.wait()
