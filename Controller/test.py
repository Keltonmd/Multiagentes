import sim
import time
import math

sim.simxFinish(-1)  # Close all connections
# Start a connection to CoppeliaSim
client_id = sim.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

if client_id != -1:
    print("Conectado com o coppelia:", client_id)
    # Start the simulation
    
    return_code, junta1 = sim.simxGetObjectHandle(client_id, 'UR3_joint1', sim.simx_opmode_blocking)
    
    return_code, junta2 = sim.simxGetObjectHandle(client_id, 'UR3_joint2', sim.simx_opmode_blocking)
    
    sim.simxSetJointTargetVelocity(client_id, junta1, 1, sim.simx_opmode_blocking)
    time.sleep(3)  # esperar 3 seconds
    sim.simxSetJointTargetVelocity(client_id, junta1, 0, sim.simx_opmode_blocking)
    
    sim.simxSetJointTargetPosition(client_id, junta2, math.pi/2, sim.simx_opmode_blocking)

else:
    print("Conexão falha!")