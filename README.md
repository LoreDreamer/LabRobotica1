# Laboratorio 1 — Simulación de un Robot Móvil Diferencial en Webots

**Robótica y Sistemas Autónomos — ICI 4150-1**

## Descripción del Laboratorio

El objetivo del laboratorio es comprender la cinemática de un robot móvil diferencial mediante simulación en Webots. Se controla un robot tipo **e-puck**, manipulando directamente las velocidades de sus ruedas para generar distintos movimientos: avance recto, curvas y rotación en el lugar. Además, se incorporan perturbaciones aleatorias para simular variaciones reales en los actuadores.

## Cómo Ejecutar la Simulación 

1. Abrir **Webots**.  
2. Cargar el mundo que contiene el robot e-puck.  
3. Abrir la carpeta: controllers/my_controller/
4. Seleccionar el archivo `my_controller.py` como controlador del robot.  
5. Ejecutar la simulación presionando **Play**.  
6. Controlar el robot utilizando el teclado.


## 🕹️ Control del Robot — Código Utilizado

El controlador fue desarrollado aplicando la cinemática diferencial del robot e-puck, asignando velocidades independientes a las ruedas izquierda y derecha.  
A partir de las teclas presionadas por el usuario, se determinan:

- la dirección del movimiento (avance, retroceso, giro o rotación en el lugar),  
- la relación de velocidades entre ambas ruedas para ejecutar la maniobra deseada,  
- y una ligera perturbación aleatoria (*bias*) que simula variabilidad real en los actuadores.

Esto permite que el robot se traslade hacia adelante, retroceda, gire suavemente o rote sobre su propio eje, dependiendo del comando ingresado.


| Tecla | Acción |
|-------|--------|
| **W** | Avanzar |
| **S** | Retroceder |
| **A** | Girar izquierda |
| **D** | Girar derecha |
| **Q** | Rotación en el lugar (izquierda) |
| **E** | Rotación en el lugar (derecha) |


## Resultados Obtenidos
A continuación se presentan los resultados obtenidos durante la ejecución del controlador. Cada subsección incluye un video demostrativo acompañado de una breve descripción del comportamiento observado

### Movimiento en línea recta
> https://github.com/user-attachments/assets/10d96754-e5f9-4f89-af2a-587dbe389bfb

En este experimento, el robot e-puck se desplaza en línea recta manteniendo ambas ruedas con la misma velocidad angular. El movimiento es estable y simétrico, sin desviaciones laterales, lo que demuestra que el modelo de control permite generar un desplazamiento lineal uniforme y preciso.


### Movimiento en curva
> https://github.com/user-attachments/assets/d2054e52-153c-4a10-bea2-1ce381865f4d

El robot ejecuta una trayectoria curva modificando la relación de velocidades entre la rueda izquierda y la derecha. Se observa cómo el e-puck genera una curva suave y continua, validando la correcta implementación del modelo cinemático para trayectorias no lineales.

### Movimiento circular
> https://github.com/user-attachments/assets/452e1c00-0b45-49fb-8ebb-e531f510b528

En este caso, el robot realiza un movimiento circular aplicando una diferencia constante entre las velocidades de ambas ruedas. La trayectoria describe un arco cerrado y uniforme, evidenciando la capacidad del controlador para generar curvas de radio constante, característica del movimiento circular.


## Análisis
Las conclusiones presentadas se construyen a partir de las pruebas realizadas al robot e-puck, evaluando cómo las configuraciones de velocidad en sus ruedas influyen directamente en la trayectoria descrita y observando los efectos que cada ajuste produce en los distintos tipos de movimiento ejecutados durante la simulación.

### 1. ¿Qué ocurre cuando ambas ruedas tienen la misma velocidad?
Cuando ambas ruedas giran a la misma velocidad y en el mismo sentido, el robot avanza en línea recta. No existe diferencia de velocidad que genere rotación, por lo que la trayectoria es rectilínea y estable.

### 2. ¿Cómo cambia la trayectoria cuando las velocidades son diferentes?
Cuando las ruedas se mueven a distintas velocidades, el robot describe una trayectoria curva. La dirección del giro depende de qué rueda sea más rápida: si la derecha tiene mayor velocidad, el robot gira hacia la izquierda; si la izquierda es más rápida, gira hacia la derecha. La diferencia entre ambas velocidades determina el radio de la curva.

### 3. ¿Qué ocurre cuando una rueda gira en sentido opuesto a la otra?
Si una rueda avanza y la otra retrocede, el robot gira sobre su propio eje. No hay desplazamiento lineal, solo rotación en el lugar, esto permite cambiar de orientación sin moverse hacia adelante o atrás.

### 4. ¿Qué tipo de movimiento permite dibujar un círculo?
Para generar una trayectoria circular, el robot debe mantener una diferencia constante entre las velocidades de ambas ruedas. Esa diferencia fija también fija el radio del círculo: una diferencia grande produce un círculo pequeño, y una diferencia menor produce un círculo más amplio.
