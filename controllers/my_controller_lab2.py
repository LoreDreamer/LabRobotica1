import os
import csv
import atexit
from controller import Robot

# Parámetros físicos y de control
WHEEL_RADIUS = 0.0205 
SPEED = 3.0
ALPHA = 0.2  

# CONFIGURACIÓN DE TIEMPO
robot = Robot()
timestep_ms = int(robot.getBasicTimeStep())
TS = timestep_ms / 1000.0  
FS = 1.0 / TS               

# VARIABLES MATEMÁTICAS DE KALMAN
Q = 0.0005   
R = 0.0025   
P = 0.01     

# UMBRALES
ENTER_DISTANCE = 0.075  
EXIT_DISTANCE = 0.105   

# Hardware
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf')); right_motor.setPosition(float('inf'))
left_encoder = robot.getDevice('left wheel sensor'); left_encoder.enable(timestep_ms)
right_encoder = robot.getDevice('right wheel sensor'); right_encoder.enable(timestep_ms)

prev_l = left_encoder.getValue()
prev_r = right_encoder.getValue()

sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
distance_sensors = {name: robot.getDevice(name) for name in sensor_names}
for s in distance_sensors.values(): s.enable(timestep_ms)

# RUTA DE GUARDADO
save_dir = r"C:\Users\TU_USUARIO\Documents\WebotsData"
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

csv_path = os.path.join(save_dir, "resultados_laboratorio.csv")
csv_file = open(csv_path, mode='w', newline='')
writer = csv.writer(csv_file)
# Cabeceras completas para análisis 
writer.writerow(["time", "raw_ps0", "raw_ps7", "l_encoder", "r_encoder", "z_procesado", "simple_filt", "kalman_est", "turning", "decision"])
atexit.register(lambda: csv_file.close())

def proximity_to_distance(raw_value):
    return max(0.0, min(0.20 / (1.0 + (raw_value / 80.0)), 0.20))

kalman_estimate = None  
simple_filt = None
turning = False
turn_direction = "turn_left"
target_wheel_rotation = 1.3 
accumulated_turn_rotation = 0.0
safety_timer = 0 

while robot.step(timestep_ms) != -1:
    time = robot.getTime()
    
    # Lectura de sensores 
    ps_values = {name: distance_sensors[name].getValue() for name in sensor_names}
    l_theta, r_theta = left_encoder.getValue(), right_encoder.getValue()
    
    # Cálculos
    front_max = max(ps_values["ps0"], ps_values["ps1"], ps_values["ps6"], ps_values["ps7"])
    z = proximity_to_distance(front_max)
    delta_s = WHEEL_RADIUS * ((l_theta - prev_l) + (r_theta - prev_r)) / 2.0
    delta_l = abs(l_theta - prev_l)
    prev_l, prev_r = l_theta, r_theta
    
    # FILTRO SIMPLE¿
    if simple_filt is None: simple_filt = z
    simple_filt = (ALPHA * z) + ((1.0 - ALPHA) * simple_filt)
    
    # FILTRO DE KALMAN 
    if kalman_estimate is None: kalman_estimate = z
    
    if not turning:
        # Predicción 
        x_pred = max(0.0, min(kalman_estimate - delta_s, 0.20))
        P_pred = P + Q
        # Corrección 
        K = P_pred / (P_pred + R)
        kalman_estimate = max(0.0, min(x_pred + K * (z - x_pred), 0.20))
        P = (1.0 - K) * P_pred
    else:
        P = 0.05 

    # LÓGICA DE NAVEGACIÓN 
    decision = "forward"
    if turning:
        accumulated_turn_rotation += delta_l
        safety_timer += 1 
        if (accumulated_turn_rotation >= target_wheel_rotation and z >= EXIT_DISTANCE) or safety_timer > 500:
            turning = False
            accumulated_turn_rotation = 0.0
            safety_timer = 0
            kalman_estimate = z 
            decision = "normal_exit"
    else:
        if kalman_estimate <= ENTER_DISTANCE:
            turning = True
            accumulated_turn_rotation = 0.0
            safety_timer = 0
            # Selección de giro con sensores laterales 
            if ps_values["ps5"] > ps_values["ps2"]:
                turn_direction = "turn_right"
            else:
                turn_direction = "turn_left"
            decision = "start_" + turn_direction

    # Motores
    if turning:
        v = 2.5 if turn_direction == "turn_right" else -2.5
        left_motor.setVelocity(-v); right_motor.setVelocity(v)
    else:
        left_motor.setVelocity(SPEED); right_motor.setVelocity(SPEED)
    
    # Registro de todas las señales necesarias para el informe 
    writer.writerow([time, ps_values["ps0"], ps_values["ps7"], l_theta, r_theta, z, simple_filt, kalman_estimate, int(turning), decision])