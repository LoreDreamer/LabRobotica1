from controller import Robot, Keyboard
import random

robot = Robot()
timestep = int(robot.getBasicTimeStep())
maxSpeed = 6.28

keyboard = Keyboard()
keyboard.enable(timestep)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(timestep) != -1:
    
    left_speed = 0.0
    right_speed = 0.0

    bias = random.uniform(-maxSpeed * 0.1, maxSpeed * 0.1)
    key = keyboard.getKey()

    if key == ord('W') or key == ord('w'):
        # adelante
        speed = maxSpeed + bias
        left_speed = speed
        right_speed = speed

    elif key == ord('S') or key == ord('s'):
        # atras
        speed = -maxSpeed + bias
        left_speed = speed
        right_speed = speed

    elif key == ord('A') or key == ord('a'):
        # giro izquierda
        left_speed = (maxSpeed / 4) + bias
        right_speed = maxSpeed - bias

    elif key == ord('D') or key == ord('d'):
        # giro derecha
        left_speed = maxSpeed + bias
        right_speed = (maxSpeed / 4) - bias

    elif key == ord('Q') or key == ord('q'):
        # giro en lugar a izquierda
        speed = (maxSpeed / 2) + bias
        left_speed = -speed
        right_speed = speed

    elif key == ord('E') or key == ord('e'):
        # giro en lugar a derecha
        speed = (maxSpeed / 2) + bias
        left_speed = speed
        right_speed = -speed

    else:
        # no mover
        left_speed = 0.0
        right_speed = 0.0
    
    # evitar errores
    left_speed = max(-maxSpeed, min(left_speed, maxSpeed))
    right_speed = max(-maxSpeed, min(right_speed, maxSpeed))

    # aplicar velocidad
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # velocidad linear y angular plus prints
    linear_velocity = (left_speed + right_speed) / 2
    angular_velocity = (right_speed - left_speed) / 0.071

    if linear_velocity != 0.0 or angular_velocity != 0.0:
        print(f"Velocidad rueda izq: {left_speed:.2f} - Velocidad rueda der: {right_speed:.2f}")
        print(f"Velocidad lineal: {linear_velocity:.2f} - Velocidad angular: {angular_velocity:.2f}")