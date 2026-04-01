from controller import Robot, Keyboard

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
    
    key = keyboard.getKey()
    
    while key != -1:
        # Avanzar / Retroceder
        if key == ord('W') or key == ord('w'):
            left_speed += maxSpeed
            right_speed += maxSpeed
        elif key == ord('S') or key == ord('s'):
            left_speed -= maxSpeed
            right_speed -= maxSpeed
            
        # Girar Izquierda / Derecha y diagonal
        elif key == ord('A') or key == ord('a'):
            left_speed -= maxSpeed / 4
            right_speed += maxSpeed / 4
        elif key == ord('D') or key == ord('d'):
            left_speed += maxSpeed / 4
            right_speed -= maxSpeed / 4
            
        key = keyboard.getKey()
     
    if (left_speed > maxSpeed):
        left_speed = maxSpeed
    elif (right_speed > maxSpeed):
        right_speed = maxSpeed
        
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    linear_velocity = (left_motor.getVelocity() + right_motor.getVelocity()) / 2
    angular_velocity = (right_motor.getVelocity() - left_motor.getVelocity())/ 0.071
    
    if (linear_velocity == 0.0 and angular_velocity == 0.0):
        pass
    else:
        print(f"Velocidad rueda izq: {left_speed} || Velocidad rueda der: {right_speed}")
        print(f"Velocidad linear: {linear_velocity} m/s || Velocidad angular: {angular_velocity} r/s")
 