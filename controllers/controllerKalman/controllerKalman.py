from controller import Robot
import csv
import os
import atexit

robot = Robot()
timestep = int(robot.getBasicTimeStep())

Ts = timestep / 1000.0
fs = 1.0 / Ts

# ============================================================
# Configuración general
# ============================================================

MAX_SPEED = 6.28

FORWARD_SPEED = 3.0
TURN_SPEED = 2.5

WHEEL_RADIUS = 0.02  # metros, radio aproximado rueda e-puck

# Filtro
ALPHA = 0.2

# Valores Kalman
Q = 0.0005   # incertidumbre de predicción
R = 0.0025   # incertidumbre de medición
P = 0.01     # incertidumbre inicial

# Conversión aproximada desde proximidad cruda a distancia pseudo-métrica.
MAX_DISTANCE = 0.20
PROX_SCALE = 80.0

# Umbral para girar
RAW_FRONT_ENTER_THRESHOLD = 120.0

# Umbral para dejar de girar
RAW_FRONT_EXIT_THRESHOLD = 80.0

# Distancias Kalman para girar y dejar de girar
KALMAN_ENTER_DISTANCE = 0.075
KALMAN_EXIT_DISTANCE = 0.105

turning = False
turn_direction = "turn_left" # Por default

# ============================================================
# Carpetas y CSV
# ============================================================

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "data")
os.makedirs(DATA_DIR, exist_ok=True)

csv_path = os.path.join(DATA_DIR, "fase2_kalman_navigation.csv")
csv_file = open(csv_path, "w", newline="")
atexit.register(csv_file.close)

writer = csv.writer(csv_file)

# ============================================================
# Motores
# ============================================================

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# ============================================================
# Encoders
# ============================================================

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

prev_left_encoder = None
prev_right_encoder = None

# ============================================================
# Sensores de proximidad IR
# ============================================================

sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]
distance_sensors = {}

for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    distance_sensors[name] = sensor

# ============================================================
# Variables de filtros
# ============================================================

front_filtered = None # valores raw de sensores filtrados
kalman_estimate = None # estimación conseguida de Kalman

# ============================================================
# Funciones auxiliares
# ============================================================

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def proximity_to_distance(proximity):
    """
    Convierte proximidad cruda del e-puck a una distancia aproximada.
    """
    distance = MAX_DISTANCE / (1.0 + (proximity / PROX_SCALE))
    return clamp(distance, 0.0, MAX_DISTANCE)


def set_wheel_speeds(left_speed, right_speed):
    left_speed = clamp(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = clamp(right_speed, -MAX_SPEED, MAX_SPEED)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    return left_speed, right_speed


def decide_turn_direction(ps2, ps5):
    """
    ps5: lado izquierdo
    ps2: lado derecho

    Si hay obstáculo por la izquierda, gira a la derecha.
    Si hay obstáculo por la derecha, gira a la izquierda.
    """
    if ps5 > ps2:
        return "turn_right"
    else:
        return "turn_left"


# ============================================================
# Encabezado CSV
# ============================================================

writer.writerow([
    "time",
    "Ts",
    "fs",

    "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7",

    "front_raw",
    "front_max",
    "front_filtered",
    "front_distance_measurement",

    "left_encoder",
    "right_encoder",
    "delta_left_theta",
    "delta_right_theta",
    "delta_left_s",
    "delta_right_s",
    "delta_s",

    "x_pred",
    "P_pred",
    "K",
    "kalman_estimate",
    "P",

    "obstacle_enter",
    "obstacle_still_near",
    "turning",
    "turn_direction",

    "left_speed",
    "right_speed",
    "decision"
])

# ============================================================
# Loop principal
# ============================================================

step_count = 0

while robot.step(timestep) != -1:
    step_count += 1
    time = robot.getTime()

    # ------------------------------------------------------------
    # 1. Leer sensores crudos
    # ------------------------------------------------------------

    ps_values = {
        name: distance_sensors[name].getValue()
        for name in sensor_names
    }

    ps0 = ps_values["ps0"]
    ps1 = ps_values["ps1"]
    ps2 = ps_values["ps2"]
    ps3 = ps_values["ps3"]
    ps4 = ps_values["ps4"]
    ps5 = ps_values["ps5"]
    ps6 = ps_values["ps6"]
    ps7 = ps_values["ps7"]

    # Frontales y frontales diagonales
    front_raw = (ps0 + ps1 + ps6 + ps7) / 4.0
    front_max = max(ps0, ps1, ps6, ps7)

    # filtro alpha
    if front_filtered is None:
        front_filtered = front_raw
    else:
        front_filtered = ALPHA * front_raw + (1.0 - ALPHA) * front_filtered

    z = proximity_to_distance(front_filtered)

    # encoders
    left_encoder_value = left_encoder.getValue()
    right_encoder_value = right_encoder.getValue()

    if prev_left_encoder is None:
        prev_left_encoder = left_encoder_value
        prev_right_encoder = right_encoder_value

    delta_left_theta = left_encoder_value - prev_left_encoder
    delta_right_theta = right_encoder_value - prev_right_encoder

    prev_left_encoder = left_encoder_value
    prev_right_encoder = right_encoder_value

    # s = r * theta
    delta_left_s = WHEEL_RADIUS * delta_left_theta
    delta_right_s = WHEEL_RADIUS * delta_right_theta
    delta_s = (delta_left_s + delta_right_s) / 2.0


    # Kalman
    if kalman_estimate is None:
        kalman_estimate = z
        x_pred = kalman_estimate
        P_pred = P
        K = 0.0
    else:
        x_pred = kalman_estimate - delta_s
        x_pred = clamp(x_pred, 0.0, MAX_DISTANCE)

        P_pred = P + Q
        K = P_pred / (P_pred + R)

        kalman_estimate = x_pred + K * (z - x_pred)
        kalman_estimate = clamp(kalman_estimate, 0.0, MAX_DISTANCE)

        P = (1.0 - K) * P_pred

    # Navegación reactiva
    obstacle_enter = (
        kalman_estimate <= KALMAN_ENTER_DISTANCE
        or front_max >= RAW_FRONT_ENTER_THRESHOLD
    )

    obstacle_still_near = (
        kalman_estimate <= KALMAN_EXIT_DISTANCE
        or front_max >= RAW_FRONT_EXIT_THRESHOLD
    )

    if not turning:
        if obstacle_enter:
            turning = True
            turn_direction = decide_turn_direction(ps2, ps5)

            if turn_direction == "turn_right":
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED
                decision = "start_turn_right"
            else:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED
                decision = "start_turn_left"
        else:
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED
            decision = "forward"

    else:
        if obstacle_still_near:
            if turn_direction == "turn_right":
                left_speed = TURN_SPEED
                right_speed = -TURN_SPEED
                decision = "turning_right"
            else:
                left_speed = -TURN_SPEED
                right_speed = TURN_SPEED
                decision = "turning_left"
        else:
            turning = False
            left_speed = FORWARD_SPEED
            right_speed = FORWARD_SPEED
            decision = "resume_forward"

    left_speed, right_speed = set_wheel_speeds(left_speed, right_speed)

    # Guardar datos
    writer.writerow([
        time,
        Ts,
        fs,

        ps0, ps1, ps2, ps3, ps4, ps5, ps6, ps7,

        front_raw,
        front_max,
        front_filtered,
        z,

        left_encoder_value,
        right_encoder_value,
        delta_left_theta,
        delta_right_theta,
        delta_left_s,
        delta_right_s,
        delta_s,

        x_pred,
        P_pred,
        K,
        kalman_estimate,
        P,

        obstacle_enter,
        obstacle_still_near,
        turning,
        turn_direction,

        left_speed,
        right_speed,
        decision
    ])

    csv_file.flush()

    # ------------------------------------------------------------
    # 8. Debug en consola
    # ------------------------------------------------------------

    if step_count % 10 == 0:
        print(
            "front_raw:",
            round(front_raw, 2),
            "| front_max:",
            round(front_max, 2),
            "| z:",
            round(z, 4),
            "| kalman:",
            round(kalman_estimate, 4),
            "| enter:",
            obstacle_enter,
            "| near:",
            obstacle_still_near,
            "| turning:",
            turning,
            "| dir:",
            turn_direction,
            "| decision:",
            decision
        )


# valores de ps0-7 / front-raw: lecturas crudas
# front-filtered: filtrada
# kalman-estimate: fusión sensorial