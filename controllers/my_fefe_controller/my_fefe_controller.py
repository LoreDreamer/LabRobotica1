import os
import csv
import atexit
import math
from pathlib import Path
from controller import Robot


#PARÁMETROS DEL ROBOT Y MUNDO

WHEEL_RADIUS = 0.0205 
AXLE_LENGTH = 0.052 
SPEED = 3.0


MAPA_GRILLA = [
    [0, 0, 0, 0, 0], 
    [0, 1, 1, 0, 0], 
    [0, 0, 1, 0, 0], 
    [0, 0, 0, 1, 0], 
    [0, 0, 0, 0, 0]  
]
INICIO = (4, 0) # Abajo a la izquierda
META = (0, 4)   # Arriba a la derecha


# ALGORITMO A* 

def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def calcular_ruta_astar(mapa, inicio, meta):
    filas = len(mapa)
    columnas = len(mapa[0])
    abiertos = [inicio]
    padres = {}
    g_cost = {inicio: 0}
    movimientos = [(-1, 0), (1, 0), (0, -1), (0, 1)] 
    
    while abiertos:
        actual = min(abiertos, key=lambda n: g_cost[n] + heuristica(n, meta))
        if actual == meta:
            ruta = []
            while actual in padres:
                ruta.append(actual)
                actual = padres[actual]
            return ruta[::-1] 
            
        abiertos.remove(actual)
        for dy, dx in movimientos:
            vecino = (actual[0] + dy, actual[1] + dx)
            if (0 <= vecino[0] < filas and 0 <= vecino[1] < columnas and mapa[vecino[0]][vecino[1]] == 0):
                nuevo_costo = g_cost[actual] + 1
                if vecino not in g_cost or nuevo_costo < g_cost[vecino]:
                    g_cost[vecino] = nuevo_costo
                    padres[vecino] = actual
                    if vecino not in abiertos:
                        abiertos.append(vecino)
    return [] 

def convertir_nodo_a_coordenadas(nodo):
    fila, col = nodo
    x = (col * 0.4) - 0.8
    y = 0.8 - (fila * 0.4)
    return x, y


# INICIALIZACIÓN DEL ROBOT

robot = Robot()
timestep_ms = int(robot.getBasicTimeStep())

# Configuración de motores y sensores
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(timestep_ms)
right_encoder.enable(timestep_ms)

sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
distance_sensors = {name: robot.getDevice(name) for name in sensor_names}
for s in distance_sensors.values(): s.enable(timestep_ms)

# Archivo de registro para el informe

CONTROLLER_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = CONTROLLER_DIR.parent.parent
DATA_DIR = PROJECT_ROOT / "data"

csv_path = DATA_DIR / "registro"
csv_file = open(csv_path, mode='w', newline='')
writer = csv.writer(csv_file)
writer.writerow(["time", "x_odom", "y_odom", "theta_odom", "modo", "distancia_obstaculo"])
atexit.register(lambda: csv_file.close())

#VARIABLES GLOBALES DE NAVEGACIÓN

prev_l = left_encoder.getValue()
prev_r = right_encoder.getValue()

# Posición inicial del robot en Webots
x_global = -0.8
y_global = -0.8
theta_global = 0.0

# Variables del filtro de Kalman
Q, R, P = 0.0005, 0.0025, 0.01
kalman_estimate = None
ENTER_DISTANCE = 0.075

# Control de estado
ruta_calculada = calcular_ruta_astar(MAPA_GRILLA, INICIO, META)
indice_ruta = 0
modo_navegacion = "SEGUIR_RUTA"

turning = False
turn_direction = "turn_left"
accumulated_turn_rotation = 0.0
safety_timer = 0

def proximity_to_distance(raw_value):
    return max(0.0, min(0.20 / (1.0 + (raw_value / 80.0)), 0.20))

print(f"Ruta calculada (Nodos): {ruta_calculada}")


# BUCLE PRINCIPAL

while robot.step(timestep_ms) != -1:
    time = robot.getTime()
    
    # Lectura de sensores
    ps_values = {name: distance_sensors[name].getValue() for name in sensor_names}
    l_theta, r_theta = left_encoder.getValue(), right_encoder.getValue()
    
    # ODOMETRÍA (Posición exacta del robot)
    delta_l = l_theta - prev_l
    delta_r = r_theta - prev_r
    prev_l, prev_r = l_theta, r_theta
    
    delta_s = WHEEL_RADIUS * (delta_l + delta_r) / 2.0
    delta_theta = WHEEL_RADIUS * (delta_r - delta_l) / AXLE_LENGTH
    
    theta_global += delta_theta
    x_global += delta_s * math.cos(theta_global)
    y_global += delta_s * math.sin(theta_global)
    
    # ESTIMACIÓN DE OBSTÁCULOS 
    front_max = max(ps_values["ps0"], ps_values["ps1"], ps_values["ps6"], ps_values["ps7"])
    z = proximity_to_distance(front_max)
    
    if kalman_estimate is None: kalman_estimate = z
    if not turning:
        x_pred = max(0.0, min(kalman_estimate - delta_s, 0.20))
        P_pred = P + Q
        K = P_pred / (P_pred + R)
        kalman_estimate = max(0.0, min(x_pred + K * (z - x_pred), 0.20))
        P = (1.0 - K) * P_pred
    else:
        P = 0.05 

    #MÁQUINA DE ESTADOS: ¿Evadir o Seguir Ruta?
    if indice_ruta >= len(ruta_calculada):
        print("¡META ALCANZADA!")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        break

    # Reacción: Si hay algo en frente, cambia a modo evasión
    if kalman_estimate <= ENTER_DISTANCE and modo_navegacion == "SEGUIR_RUTA":
        modo_navegacion = "EVADIENDO"
        turning = True
        accumulated_turn_rotation = 0.0
        safety_timer = 0
        turn_direction = "turn_right" if ps_values["ps5"] > ps_values["ps2"] else "turn_left"

    left_speed = 0.0
    right_speed = 0.0

    # COMPORTAMIENTO: EVADIR OBSTÁCULO MÓVIL/IMPREVISTO
    if modo_navegacion == "EVADIENDO":
        accumulated_turn_rotation += abs(delta_l)
        safety_timer += 1
        
        # Rotar en su eje
        v_giro = 2.0 if turn_direction == "turn_right" else -2.0
        left_speed = -v_giro
        right_speed = v_giro
        
        # Condición de salida: Giró lo suficiente y el frente está libre
        if (accumulated_turn_rotation >= 1.5 and z >= 0.10) or safety_timer > 500:
            modo_navegacion = "SEGUIR_RUTA"
            turning = False

    # COMPORTAMIENTO: SEGUIR RUTA GLOBAL
    elif modo_navegacion == "SEGUIR_RUTA":
        objetivo_x, objetivo_y = convertir_nodo_a_coordenadas(ruta_calculada[indice_ruta])
        
        # Calcular ángulo hacia el objetivo
        angulo_objetivo = math.atan2(objetivo_y - y_global, objetivo_x - x_global)
        error_angulo = angulo_objetivo - theta_global
        error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo)) # Normalizar
        
        distancia_al_objetivo = math.sqrt((objetivo_x - x_global)**2 + (objetivo_y - y_global)**2)
        
        # Si llegó al nodo, avanzar al siguiente
        if distancia_al_objetivo < 0.05:
            indice_ruta += 1
        else:
            # Control P simple para apuntar y avanzar
            if abs(error_angulo) > 0.15: # Corregir rumbo
                if error_angulo > 0:
                    left_speed, right_speed = -SPEED*0.5, SPEED*0.5
                else:
                    left_speed, right_speed = SPEED*0.5, -SPEED*0.5
            else: # Avanzar recto
                left_speed, right_speed = SPEED, SPEED

    # Aplicar velocidades
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    # Registro de datos
    writer.writerow([time, x_global, y_global, theta_global, modo_navegacion, kalman_estimate])