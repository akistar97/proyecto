# 🏎️ F1TENTH - Controlador: Follow the Gap

Este proyecto tiene un controlador para un recorrido autonomo de un vehículo en el simulado de **F1TENTH** utilizando **ROS 2**. El objetivo es completar 10 vueltas en la psita de Budapest_map del simulador sin colisionar y registrar los tiempos por vuelta, todo utilizando el enfoque **Follow the Gap**.

---

## 🚗 Enfoque utilizado: Follow the Gap

El algoritmo **Follow the Gap** consiste en:

- Analizar las lecturas del sensor **LiDAR** para detectar obstáculos.
- Eliminar una "burbuja de seguridad" alrededor del obstáculo más cercano.
- Buscar el **hueco (gap)** más amplio disponible.
- Calcular el ángulo hacia el centro del gap.
- Dirigir el vehículo hacia ese punto.

Además:
- Se usa la **odometría** para registrar el número de vueltas.
- Se mide y muestra el tiempo por vuelta.
- Se ajusta la **velocidad dinámicamente**:
  - Mayor velocidad en rectas (`6.5 m/s`).
  - Menor velocidad en curvas (`2.0 m/s`).

---

## Estructura esperada del paquete

    gap_follow_controller/
    ├── gap_follow_controller/
    │ ├── init.py
    │ └── gap_follow.py # Código principal del controlador
    ├── package.xml # Descripción del paquete ROS 2
    └── setup.py # Configuración para instalación


---

## 📁 Estructura del código gap_follow.py

       
        
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    import numpy as np
    import time
    import math

    class GapFollow(Node):
        def __init__(self):
            super().__init__('gap_follow_node')

        # Subscripción al LiDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        # Subscripción a la odometría para contar vueltas
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        # Publicador de comandos
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        # Variables para conteo de vueltas
        self.start_pos = None
        self.lap_counter = 0
        self.lap_start_time = time.time()
        self.lap_times = []
        self.min_lap_interval = 5.0  # para evitar falsos positivos
        self.last_lap_time = time.time()

        self.get_logger().info('🚗 Nodo Follow the Gap iniciado')

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges = np.clip(ranges, msg.range_min, msg.range_max)

        # Evita obstáculos con burbuja
        closest_idx = np.argmin(ranges)
        bubble_radius = 10
        start = max(0, closest_idx - bubble_radius)
        end = min(len(ranges), closest_idx + bubble_radius)
        ranges[start:end] = 0.0

        # Encuentra hueco más grande
        gap_start, gap_size = self.find_largest_gap(ranges)
        best_point = gap_start + gap_size // 2
        angle = msg.angle_min + best_point * msg.angle_increment

        # Control dinámico de velocidad basado en el ángulo
        angle_threshold = 0.1  # radianes, ajustar según necesidad
        if abs(angle) < angle_threshold:
            speed = 6.5  # recta
        else:
            speed = 2.0  # curva

        # Publica el movimiento
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.publisher.publish(drive_msg)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_pos is None:
            self.start_pos = (x, y)
            self.get_logger().info(f"📍 Punto de inicio registrado: ({x:.2f}, {y:.2f})")
            return

        dist_to_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])

        if dist_to_start < 1.0:  # dentro del radio de vuelta
            current_time = time.time()
            if (current_time - self.last_lap_time) > self.min_lap_interval:
                lap_time = current_time - self.lap_start_time
                self.lap_times.append(lap_time)
                self.lap_counter += 1
                self.last_lap_time = current_time
                self.lap_start_time = current_time
                self.get_logger().info(f"🏁 Vuelta {self.lap_counter}: {lap_time:.2f} s")

                if self.lap_counter >= 10:
                    self.get_logger().info("✅ Completadas 10 vueltas")
                    self.get_logger().info(f"⏱️ Tiempos por vuelta: {self.lap_times}")
                    rclpy.shutdown()

    def find_largest_gap(self, ranges):
        max_count = 0
        max_start = 0
        current_count = 0
        current_start = 0

        for i in range(len(ranges)):
            if ranges[i] > 1.5:
                if current_count == 0:
                    current_start = i
                current_count += 1
                if current_count > max_count:
                    max_count = current_count
                    max_start = current_start
            else:
                current_count = 0

        return max_start, max_count


    def main(args=None):
        rclpy.init(args=args)
        node = GapFollow()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

---

## 🔍 Análisis detallado del código

## 1. Importación de librerías

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from ackermann_msgs.msg import AckermannDriveStamped
    import numpy as np
    import time
    import math
    
Función:
Importa las bibliotecas necesarias:

rclpy: Cliente de ROS 2 en Python.

Node: Para crear nodos ROS.

LaserScan: Tipo de mensaje del sensor LiDAR.

Odometry: Posición y orientación del vehículo.

AckermannDriveStamped: Mensaje para controlar dirección y velocidad.

numpy, time, math: Librerías estándar para cálculos.

Resultado:
Disponibilidad de todas las funciones necesarias para usar ROS 2 y procesamiento numérico.

## 2. Clase del nodo principal

    class GapFollow(Node):
        def __init__(self):
            super().__init__('gap_follow_node')
Función:
Crea una clase llamada GapFollow que extiende Node, lo cual permite usar funcionalidades ROS.

Resultado:
Se inicializa un nodo llamado "gap_follow_node" en la red de ROS 2.

## 3. Subscripción al LiDAR

    self.subscription = self.create_subscription(
        LaserScan,
        '/scan',
        self.laser_callback,
        10
    )
    
Función:
Escucha datos del sensor LiDAR en el tópico /scan.

Entrada esperada:
Mensajes de tipo LaserScan, enviados por el simulador F1TENTH.

Resultado:
Cada vez que llegan datos del LiDAR, se llama automáticamente a laser_callback(msg).

## 4. Subscripción a la odometría

    self.odom_subscription = self.create_subscription(
        Odometry,
        '/ego_racecar/odom',
        self.odom_callback,
        10
    )
    
Función:
Escucha datos de posición y orientación del carro.

Entrada esperada:
Mensajes de tipo Odometry, que describen la ubicación del vehículo en el mapa simulado.

Resultado:
Se llama a odom_callback(msg) cada vez que llega nueva información de odometría.

## 5. Publicador del controlador

    self.publisher = self.create_publisher(
        AckermannDriveStamped,
        '/drive',
        10
    )
    
Función:
Crea un publicador que enviará comandos de velocidad y dirección.

Entrada esperada:
Mensajes de tipo AckermannDriveStamped generados por el código.

Resultado:
Envía las instrucciones al vehículo para moverse.

## 6. Inicialización de variables

    self.start_pos = None
    self.lap_counter = 0
    self.lap_start_time = time.time()
    self.lap_times = []
    self.min_lap_interval = 5.0
    self.last_lap_time = time.time()
    
Función:
Guarda información importante para contar vueltas y medir tiempos.

start_pos: Guarda el punto de partida.

lap_counter: Contador de vueltas completadas.

lap_times: Lista de tiempos por vuelta.

min_lap_interval: Tiempo mínimo entre dos vueltas para evitar errores.

last_lap_time: Momento en que terminó la última vuelta.


## 7. Callback del LiDAR – lógica principal de navegación

    def laser_callback(self, msg):
    
Entrada esperada:
Un mensaje LaserScan del sensor LiDAR.

Resultado esperado:
Calcula la mejor dirección para conducir el vehículo evitando obstáculos y ajustando la velocidad según el entorno.

a. Procesamiento de datos LiDAR

    ranges = np.array(msg.ranges)
    ranges = np.clip(ranges, msg.range_min, msg.range_max)
    Convierte las distancias en un arreglo NumPy.

Elimina valores fuera del rango confiable del sensor.

b. Eliminar burbuja de seguridad

    closest_idx = np.argmin(ranges)
    bubble_radius = 10
    start = max(0, closest_idx - bubble_radius)
    end = min(len(ranges), closest_idx + bubble_radius)
    ranges[start:end] = 0.0

Encuentra el punto más cercano (posible obstáculo).

Crea una “burbuja” eliminando datos cercanos a ese punto.

Esto evita que el vehículo intente atravesar huecos muy estrechos.

c. Encontrar el hueco más grande

    gap_start, gap_size = self.find_largest_gap(ranges)
    best_point = gap_start + gap_size // 2
    angle = msg.angle_min + best_point * msg.angle_increment

Usa la función auxiliar find_largest_gap() para encontrar el hueco más grande.

Calcula el ángulo hacia el centro de ese hueco.

d. Ajustar velocidad según dirección

    angle_threshold = 0.1
    if abs(angle) < angle_threshold:
        speed = 6.5  # más rápido en rectas
    else:
        speed = 2.0  # más lento en curvas

Si el ángulo es pequeño (recta), acelera.

Si el ángulo es grande (curva), reduce la velocidad.

e. Publicar comando de movimiento

    drive_msg = AckermannDriveStamped()
    drive_msg.drive.speed = speed
    drive_msg.drive.steering_angle = angle
    self.publisher.publish(drive_msg)

Crea el mensaje de conducción y lo publica en /drive.

## 8. Callback de odometría – detección de vueltas

    def odom_callback(self, msg):

Entrada esperada:
Mensaje Odometry con la posición del vehículo.

a. Guardar punto inicial

    if self.start_pos is None:
        self.start_pos = (x, y)

Se ejecuta solo una vez.

Guarda la posición donde comienza el vehículo para detectar vueltas.

b. Verificar si el vehículo regresó al inicio

    dist_to_start = math.hypot(x - self.start_pos[0], y - self.start_pos[1])
    if dist_to_start < 1.0:

Calcula la distancia desde la posición actual al punto de inicio.

Si está lo suficientemente cerca, cuenta como una vuelta.

c. Contar vueltas y registrar tiempo

    if (current_time - self.last_lap_time) > self.min_lap_interval:
        lap_time = current_time - self.lap_start_time
        self.lap_times.append(lap_time)
        self.lap_counter += 1
        
Guarda el tiempo de la vuelta.

Aumenta el contador.

Reinicia temporizadores.

d. Finalizar al completar 10 vueltas

    if self.lap_counter >= 10:
        rclpy.shutdown()
        
Si ya se completaron 10 vueltas, se cierra el nodo.

## 9. Buscar el hueco más grande

    def find_largest_gap(self, ranges):

Entrada esperada:
Arreglo con distancias filtradas del LiDAR.

Resultado esperado:
Índice y tamaño del hueco más grande entre obstáculos.

## 10. Función principal

    def main(args=None):
        rclpy.init(args=args)
        node = GapFollow()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
        
Función:
Inicializa ROS 2.

Crea y ejecuta el nodo hasta que se completen las 10 vueltas.

Cierra el nodo correctamente.


---

### Principales componentes del código

| Función                  | Descripción |
|--------------------------|-------------|
| `laser_callback`         | Procesa datos del LiDAR, detecta el mejor hueco y publica comandos de dirección y velocidad. |
| `find_largest_gap`       | Busca el mayor hueco entre obstáculos. |
| `odom_callback`          | Detecta si el vehículo ha regresado al punto de inicio y cuenta vueltas. |
| `main()`                 | Inicializa el nodo y mantiene su ejecución. |

---

## 🛠️ Instrucciones de uso

### 📦 Requisitos

- ROS 2 (Foxy, Humble, Rolling, etc.)
- Simulador F1TENTH funcionando
- Workspace en `~/f1tenth_ws`

### 🧩 Instalación

```bash
cd ~/f1tenth_ws/src
git clone https://github.com/tu_usuario/gap_follow_controller.git
cd ~/f1tenth_ws
colcon build
source install/setup.bash
