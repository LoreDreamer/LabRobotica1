import os
import csv
import math
from controller import Robot


#PARÁMETROS DEL ROBOT
WHEEL_RADIUS = 0.0205 
AXLE_LENGTH = 0.057  
SPEED = 0.8
ENTER_DISTANCE = 0.01


#REPRESENTACIÓN DEL ENTORNO 
# 0 = Libre, 1 = Obstáculo
MAPA_GRILLA = [
    [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],  # fila 0 abajo
    [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0],
    [0,0,0,1,0,0,0,1,1,1,0,0,1,1,0],
    [0,0,0,1,0,0,0,1,1,1,0,0,1,1,0],
    [0,0,0,1,1,1,0,0,0,0,0,0,1,1,0],
    [0,0,0,0,0,0,0,0,0,1,0,0,1,1,0],
    [0,0,1,1,1,1,1,0,0,1,0,0,0,0,0],
    [0,0,1,1,1,1,1,0,0,1,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,1,0,0,1,1,1],
    [1,1,1,1,1,0,0,0,0,0,0,0,1,1,1],
    [1,1,1,1,1,0,0,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,1,1,1,0,0,0,1],
    [0,0,0,0,0,0,0,0,0,1,1,0,0,0,1],
    [0,0,0,0,0,0,0,0,0,1,1,0,0,1,1],
    [0,0,0,0,0,0,0,0,0,0,1,0,0,0,0]   # fila 14 arriba
]
# Inicio en la esquina inferior izquierda, Meta en la superior derecha
INICIO = (0, 0) 
META = (14, 14)   


CELDAS_X = 15
CELDAS_Y = 15
TAMANO_CELDA = 1.0 / 15.0

def nodo_a_coordenadas(nodo):
    fila, col = nodo
    x = (col * TAMANO_CELDA) - (CELDAS_X * TAMANO_CELDA / 2.0) + (TAMANO_CELDA / 2.0)
    y = (fila * TAMANO_CELDA) - (CELDAS_Y * TAMANO_CELDA / 2.0) + (TAMANO_CELDA / 2.0)
    return x, y

def es_giro(ruta, indice):
    if indice <= 0 or indice >= len(ruta) - 1:
        return False

    anterior = ruta[indice - 1]
    actual = ruta[indice]
    siguiente = ruta[indice + 1]

    dir1 = (actual[0] - anterior[0], actual[1] - anterior[1])
    dir2 = (siguiente[0] - actual[0], siguiente[1] - actual[1])

    return dir1 != dir2

# ALGORITMO DE PLANIFICACIÓN A*

def heuristica(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def calcular_ruta_astar(mapa, inicio, meta):
    filas, columnas = len(mapa), len(mapa[0])
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


# CONFIGURACIÓN DEL ROBOT 

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf')); right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0); right_motor.setVelocity(0.0)

left_encoder = robot.getDevice('left wheel sensor'); left_encoder.enable(timestep)
right_encoder = robot.getDevice('right wheel sensor'); right_encoder.enable(timestep)

sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
distance_sensors = {name: robot.getDevice(name) for name in sensor_names}
for s in distance_sensors.values(): s.enable(timestep)

csv_file = open("resultados_simple.csv", mode='w', newline='')
writer = csv.writer(csv_file)
writer.writerow(["time", "x_real", "y_real", "theta_real", "estado"])


# VARIABLES DE ESTADO Y ODOMETRÍA

robot.step(timestep)
prev_l = left_encoder.getValue()
prev_r = right_encoder.getValue()

# Posición inicial odométrica EXACTA a Webots 
x_global, y_global = nodo_a_coordenadas(INICIO)
theta_global = 0.0

kalman_estimate = None
P, Q, R = 0.01, 0.0005, 0.0025

ruta_calculada = calcular_ruta_astar(MAPA_GRILLA, INICIO, META)

print("Inicio:", INICIO)
print("Meta:", META)

for fila in reversed(MAPA_GRILLA):
    print(fila)

if not ruta_calculada and INICIO != META:
    print("ERROR: No se encontró ruta entre INICIO y META.")
    while robot.step(timestep) != -1:
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
indice_ruta = 0
estado_navegacion = "SEGUIR_RUTA"
rotacion_acumulada = 0.0 # Variable para contar las vueltas de celebración


print("Ruta:", ruta_calculada)

print(f"Ruta generada (Nodos): {ruta_calculada}")

for i, nodo in enumerate(ruta_calculada):
    print(i, nodo)


def imprimir_mapa(mapa, inicio, meta):
    for fila in range(len(mapa)-1, -1, -1):
        linea = ""
        for col in range(len(mapa[0])):
            if (fila, col) == inicio:
                linea += "R "
            elif (fila, col) == meta:
                linea += "M "
            elif mapa[fila][col] == 1:
                linea += "█ "
            else:
                linea += ". "
        print(linea)


def proximity_to_distance(raw_value):
    return max(0.0, min(0.20 / (1.0 + (raw_value / 80.0)), 0.20))


# BUCLE PRINCIPAL

while robot.step(timestep) != -1:
    time = robot.getTime()
    
    # ODOMETRÍA 
    l_theta, r_theta = left_encoder.getValue(), right_encoder.getValue()
    delta_s_l = WHEEL_RADIUS * (l_theta - prev_l)
    delta_s_r = WHEEL_RADIUS * (r_theta - prev_r)
    prev_l, prev_r = l_theta, r_theta
    
    delta_s = (delta_s_r + delta_s_l) / 2.0
    delta_theta = (delta_s_r - delta_s_l) / AXLE_LENGTH
    
    theta_global += delta_theta
    # Restringir ángulo global entre -pi y pi para mayor estabilidad
    theta_global = math.atan2(math.sin(theta_global), math.cos(theta_global)) 
    
    x_global += delta_s * math.cos(theta_global)
    y_global += delta_s * math.sin(theta_global)
    
    # SENSORES FRONTALES 
    ps_values = {n: distance_sensors[n].getValue() for n in sensor_names}
    f_max = max(ps_values["ps0"], ps_values["ps1"], ps_values["ps6"], ps_values["ps7"])
    z_sensor = proximity_to_distance(f_max)
    
    if kalman_estimate is None: kalman_estimate = z_sensor
    x_pred = kalman_estimate - delta_s
    P_pred = P + Q
    K = P_pred / (P_pred + R)
    kalman_estimate = x_pred + K * (z_sensor - x_pred)
    P = (1.0 - K) * P_pred

    # MÁQUINA DE NAVEGACIÓN
    if indice_ruta >= len(ruta_calculada):
        if estado_navegacion != "CELEBRANDO":
            print("\n=======================================================")
            print(f"¡ÉXITO! El robot ha llegado a la META en el nodo {META} ")
            print(f"Coordenadas finales: X = {x_global:.3f} m, Y = {y_global:.3f} m")
            print("=======================================================\n")
            estado_navegacion = "CELEBRANDO"
            rotacion_acumulada = 0.0
        
        # Hacer las 3 vueltas 
        rotacion_acumulada += abs(delta_theta)
        
        if rotacion_acumulada < (6 * math.pi):
            left_speed = -SPEED * 0.8
            right_speed = SPEED * 0.8
        else:
            print("¡Baile de victoria terminado! Misión cumplida.")
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            break

    elif kalman_estimate <= ENTER_DISTANCE:
        estado_navegacion = "EVASIÓN_EMERGENCIA"
    
        izquierda = max(ps_values["ps5"], ps_values["ps6"], ps_values["ps7"])
        derecha = max(ps_values["ps0"], ps_values["ps1"], ps_values["ps2"])
    
        if f_max > 120:
            left_speed = -SPEED * 0.4
            right_speed = -SPEED * 0.4
    
        elif izquierda > derecha:
            left_speed = SPEED * 0.45
            right_speed = -SPEED * 0.45
    
        else:
            left_speed = -SPEED * 0.45
            right_speed = SPEED * 0.45
    else:
        estado_navegacion = "SEGUIR_RUTA"
        objetivo_x, objetivo_y = nodo_a_coordenadas(ruta_calculada[indice_ruta])
        
        angulo_objetivo = math.atan2(objetivo_y - y_global, objetivo_x - x_global)
        error_angulo = angulo_objetivo - theta_global
        error_angulo = math.atan2(math.sin(error_angulo), math.cos(error_angulo))
        
        distancia_objetivo = math.sqrt((objetivo_x - x_global)**2 + (objetivo_y - y_global)**2)
        
        tolerancia_nodo = 0.035 if es_giro(ruta_calculada, indice_ruta) else 0.025
        
        if distancia_objetivo < tolerancia_nodo:
            indice_ruta += 1 
            left_speed, right_speed = 0.0, 0.0
            
        else:
            if abs(error_angulo) > 0.08: 
                velocidad_giro = SPEED * 0.25
                # Rotar primero hacia el objetivo sin avanzar 
                if error_angulo > 0: 
                    left_speed, right_speed = -velocidad_giro, velocidad_giro  # Girar a la izquierda
                else: 
                    left_speed, right_speed = velocidad_giro, -velocidad_giro  # Girar a la derecha
            else: 
                # Avanzar recto
                left_speed, right_speed = SPEED, SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    writer.writerow([time, x_global, y_global, theta_global, estado_navegacion])
    

csv_file.close()