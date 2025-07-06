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
