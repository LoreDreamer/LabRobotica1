import os
import json
import csv
import atexit
import math

from controller import Robot


# =============================================================================
# RUTAS DEL PROYECTO
# =============================================================================

CONTROLLER_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CONTROLLER_DIR, "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
MAPS_JSON_PATH = os.path.join(DATA_DIR, "maps", "maps.json")
CSV_PATH = os.path.join(DATA_DIR, "registro_navegacion.csv")

# =============================================================================
# PARÁMETROS
# =============================================================================

NOMBRE_MAPA = "easy" # VALOR CONFIGURABLE ENTRE "easy" y "hard"

WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.057
SPEED = 2.0
ENTER_DISTANCE = 0.005

TOLERANCIA_ANGULAR = math.radians(1.0)
TOLERANCIA_NODO_RECTA = 0.001
TOLERANCIA_NODO_GIRO = 0.002

# =============================================================================
# CONFIGURACIÓN DEL MAPA
# =============================================================================

def cargar_mapa(json_path, nombre_mapa):
    """Carga desde maps.json la grilla, inicio, meta, escala y orientación del mapa solicitado."""
    with open(json_path, mode="r", encoding="utf-8") as archivo:
        configuracion = json.load(archivo)

    if nombre_mapa not in configuracion["maps"]:
        raise KeyError(f"El mapa '{nombre_mapa}' no existe en maps.json.")

    mapa_config = configuracion["maps"][nombre_mapa]

    mapa = mapa_config["grid"]
    inicio = tuple(mapa_config["start"])
    meta = tuple(mapa_config["goal"])
    theta_inicial = float(mapa_config["initial_heading_rad"])
    tamano_celda = float(configuracion["cell_size_m"])
    centro_mapa = tuple(configuracion["map_center_m"])

    return mapa, inicio, meta, theta_inicial, tamano_celda, centro_mapa


MAPA_GRILLA, INICIO, META, THETA_INICIAL, TAMANO_CELDA, MAP_CENTER_M = cargar_mapa(
    MAPS_JSON_PATH,
    NOMBRE_MAPA,
)

CELDAS_Y = len(MAPA_GRILLA)
CELDAS_X = len(MAPA_GRILLA[0])

# =============================================================================
# CONVERSIÓN DE COORDENADAS
# =============================================================================

def nodo_a_coordenadas(nodo):
    """Convierte un nodo (columna, fila) al centro físico de su celda en metros."""
    columna, fila = nodo
    ancho_mapa = CELDAS_X * TAMANO_CELDA
    alto_mapa = CELDAS_Y * TAMANO_CELDA
    centro_x, centro_y = MAP_CENTER_M

    x = centro_x - ancho_mapa / 2.0 + (columna + 0.5) * TAMANO_CELDA
    y = centro_y + alto_mapa / 2.0 - (fila + 0.5) * TAMANO_CELDA

    return x, y

def es_giro(ruta, indice):
    """Indica si el nodo de la ruta corresponde a un cambio de dirección."""
    if indice <= 0 or indice >= len(ruta) - 1:
        return False

    anterior = ruta[indice - 1]
    actual = ruta[indice]
    siguiente = ruta[indice + 1]

    direccion_entrada = (
        actual[0] - anterior[0],
        actual[1] - anterior[1],
    )

    direccion_salida = (
        siguiente[0] - actual[0],
        siguiente[1] - actual[1],
    )

    return direccion_entrada != direccion_salida

# =============================================================================
# ALGORITMO A*
# =============================================================================

def manhattan(a, b):
    """Calcula la distancia Manhattan entre dos nodos."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def calcular_ruta_astar(mapa, inicio, meta):
    """Calcula una ruta de celdas libres desde inicio hasta meta mediante A*."""
    filas = len(mapa)
    columnas = len(mapa[0])

    def nodo_valido(nodo):
        """Comprueba que un nodo esté dentro del mapa y no sea un obstáculo."""
        columna, fila = nodo
        return (
            0 <= columna < columnas
            and 0 <= fila < filas
            and mapa[fila][columna] == 0
        )

    if not nodo_valido(inicio):
        raise ValueError(f"El inicio {inicio} es inválido o es un obstáculo.")

    if not nodo_valido(meta):
        raise ValueError(f"La meta {meta} es inválida o es un obstáculo.")

    abiertos = [inicio]
    cerrados = set()
    padres = {}
    g_cost = {inicio: 0}

    movimientos = [
        (0, -1),
        (0, 1),
        (-1, 0),
        (1, 0),
    ]

    while abiertos:
        actual = min(
            abiertos,
            key=lambda nodo: g_cost[nodo] + manhattan(nodo, meta),
        )

        if actual == meta:
            ruta = [actual]

            while actual in padres:
                actual = padres[actual]
                ruta.append(actual)

            ruta.reverse()
            return ruta

        abiertos.remove(actual)
        cerrados.add(actual)

        for delta_columna, delta_fila in movimientos:
            vecino = (
                actual[0] + delta_columna,
                actual[1] + delta_fila,
            )

            if not nodo_valido(vecino) or vecino in cerrados:
                continue

            nuevo_costo = g_cost[actual] + 1

            if vecino not in g_cost or nuevo_costo < g_cost[vecino]:
                g_cost[vecino] = nuevo_costo
                padres[vecino] = actual

                if vecino not in abiertos:
                    abiertos.append(vecino)

    return []

# =============================================================================
# FUNCIONES AUXILIARES
# =============================================================================

def imprimir_mapa(mapa, inicio, meta, ruta=None):
    """Imprime la grilla indicando inicio, meta, obstáculos y ruta planificada."""
    ruta = set(ruta or [])

    for fila in range(len(mapa)):
        linea = ""

        for columna in range(len(mapa[fila])):
            nodo = (columna, fila)

            if nodo == inicio:
                linea += "R "
            elif nodo == meta:
                linea += "M "
            elif mapa[fila][columna] == 1:
                linea += "█ "
            elif nodo in ruta:
                linea += "* "
            else:
                linea += ". "

        print(linea)

def proximity_to_distance(raw_value):
    """Convierte aproximadamente el valor del sensor infrarrojo a metros."""
    distancia = 0.20 / (1.0 + raw_value / 80.0)
    return max(0.0, min(distancia, 0.20))

# =============================================================================
# CONFIGURACIÓN DEL ROBOT
# =============================================================================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
distance_sensors = {
    nombre: robot.getDevice(nombre)
    for nombre in sensor_names
}

for sensor in distance_sensors.values():
    sensor.enable(timestep)

# =============================================================================
# ARCHIVO CSV
# =============================================================================

os.makedirs(DATA_DIR, exist_ok=True)

csv_file = open(
    CSV_PATH,
    mode="w",
    newline="",
    encoding="utf-8",
)

writer = csv.writer(csv_file)
writer.writerow([
    "time",
    "x_odom",
    "y_odom",
    "theta_odom",
    "modo",
    "distancia_obstaculo",
])

atexit.register(csv_file.close)

# =============================================================================
# VARIABLES DE NAVEGACIÓN
# =============================================================================

robot.step(timestep)

prev_l = left_encoder.getValue()
prev_r = right_encoder.getValue()

x_global, y_global = nodo_a_coordenadas(INICIO)
theta_global = THETA_INICIAL

kalman_estimate = None
P, Q, R = 0.01, 0.0005, 0.0025

ruta_calculada = calcular_ruta_astar(
    MAPA_GRILLA,
    INICIO,
    META,
)

if not ruta_calculada: raise RuntimeError(f"No se encontró una ruta entre {INICIO} y {META}.")

indice_ruta = 1
modo_navegacion = "SEGUIR_RUTA"
rotacion_acumulada = 0.0

# =============================================================================
# INFORMACIÓN INICIAL
# =============================================================================

print("Inicio:", INICIO, nodo_a_coordenadas(INICIO))
print("Meta:", META, nodo_a_coordenadas(META))

imprimir_mapa(MAPA_GRILLA, INICIO, META, ruta_calculada)
print("Ruta:", ruta_calculada)

# =============================================================================
# BUCLE PRINCIPAL
# =============================================================================

while robot.step(timestep) != -1:
    tiempo = robot.getTime()

    l_theta = left_encoder.getValue()
    r_theta = right_encoder.getValue()

    delta_phi_l = l_theta - prev_l
    delta_phi_r = r_theta - prev_r

    prev_l = l_theta
    prev_r = r_theta

    delta_s_l = WHEEL_RADIUS * delta_phi_l
    delta_s_r = WHEEL_RADIUS * delta_phi_r

    delta_s = (delta_s_r + delta_s_l) / 2.0
    delta_theta = (delta_s_r - delta_s_l) / AXLE_LENGTH

    theta_global += delta_theta
    theta_global = math.atan2(
        math.sin(theta_global),
        math.cos(theta_global),
    )

    x_global += delta_s * math.cos(theta_global)
    y_global += delta_s * math.sin(theta_global)

    ps_values = {
        nombre: distance_sensors[nombre].getValue()
        for nombre in sensor_names
    }

    front_max = max(
        ps_values["ps0"],
        ps_values["ps7"],
    )

    z_sensor = proximity_to_distance(front_max)

    if kalman_estimate is None: kalman_estimate = z_sensor

    x_pred = kalman_estimate - delta_s
    P_pred = P + Q
    K = P_pred / (P_pred + R)

    kalman_estimate = x_pred + K * (z_sensor - x_pred)
    kalman_estimate = max(0.0, min(kalman_estimate, 0.20))
    P = (1.0 - K) * P_pred

    if indice_ruta >= len(ruta_calculada):
        if modo_navegacion != "CELEBRANDO":
            modo_navegacion = "CELEBRANDO"
            rotacion_acumulada = 0.0

        rotacion_acumulada += abs(delta_theta)

        if rotacion_acumulada < 6.0 * math.pi:
            left_speed = -SPEED * 0.8
            right_speed = SPEED * 0.8
        else:
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)

            writer.writerow([
                tiempo,
                x_global,
                y_global,
                theta_global,
                modo_navegacion,
                kalman_estimate,
            ])

            break

    elif kalman_estimate <= ENTER_DISTANCE:
        modo_navegacion = "EVASIÓN_EMERGENCIA"

        izquierda = max(
            ps_values["ps5"],
            ps_values["ps6"],
            ps_values["ps7"],
        )

        derecha = max(
            ps_values["ps0"],
            ps_values["ps1"],
            ps_values["ps2"],
        )

        if front_max > 120:
            left_speed = -SPEED * 0.4
            right_speed = -SPEED * 0.4
        elif izquierda > derecha:
            left_speed = SPEED * 0.45
            right_speed = -SPEED * 0.45
        else:
            left_speed = -SPEED * 0.45
            right_speed = SPEED * 0.45

    else:
        modo_navegacion = "SEGUIR_RUTA"

        nodo_objetivo = ruta_calculada[indice_ruta]
        objetivo_x, objetivo_y = nodo_a_coordenadas(nodo_objetivo)

        diferencia_x = objetivo_x - x_global
        diferencia_y = objetivo_y - y_global

        angulo_objetivo = math.atan2(
            diferencia_y,
            diferencia_x,
        )

        error_angulo = math.atan2(
            math.sin(angulo_objetivo - theta_global),
            math.cos(angulo_objetivo - theta_global),
        )

        distancia_objetivo = math.hypot(
            diferencia_x,
            diferencia_y,
        )

        tolerancia_nodo = (
            TOLERANCIA_NODO_GIRO
            if es_giro(ruta_calculada, indice_ruta)
            else TOLERANCIA_NODO_RECTA
        )

        if distancia_objetivo < tolerancia_nodo:
            indice_ruta += 1
            left_speed = 0.0
            right_speed = 0.0

        elif abs(error_angulo) > TOLERANCIA_ANGULAR:
            velocidad_giro = SPEED * 0.25

            if error_angulo > 0.0:
                left_speed = -velocidad_giro
                right_speed = velocidad_giro
            else:
                left_speed = velocidad_giro
                right_speed = -velocidad_giro

        else:
            left_speed = SPEED
            right_speed = SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    writer.writerow([
        tiempo,
        x_global,
        y_global,
        theta_global,
        modo_navegacion,
        kalman_estimate,
    ])


left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)
csv_file.close()
