#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_nav")

        # Publisher do comando de movimento
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Lista de 5 waypoints (x, y)
        # Coloque suas coordenadas reais aqui
        self.waypoints = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 1.0),
            (0.0, 0.0),
        ]

        # Posição inicial (se não estiver usando odometria real)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0   # orientação aproximada

        self.navigate()

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        self.publish_twist(0.0, 0.0)

    def rotate_to_angle(self, target_theta):
        """Gira até alcançar o ângulo desejado"""
        while rclpy.ok() and abs(target_theta - self.theta) > 0.05:
            direction = 1.0 if target_theta > self.theta else -1.0
            self.publish_twist(0.0, 0.4 * direction)
            self.theta += 0.02 * direction
            time.sleep(0.02)
        self.stop()

    def go_straight(self, distance):
        """Anda para frente até completar a distância"""
        walked = 0.0
        while rclpy.ok() and walked < distance:
            self.publish_twist(0.15, 0.0)
            walked += 0.015
            time.sleep(0.02)
        self.stop()

    def navigate(self):
        self.get_logger().info("Iniciando navegação por 5 waypoints...")

        for i, (wx, wy) in enumerate(self.waypoints):
            self.get_logger().info(f"Indo para waypoint {i+1}: {wx}, {wy}")

            dx = wx - self.x
            dy = wy - self.y

            # Ângulo até o waypoint
            target_theta = math.atan2(dy, dx)

            # Distância até o waypoint
            distance = math.sqrt(dx*dx + dy*dy)

            # 1) Girar até o ângulo correto
            self.rotate_to_angle(target_theta)

            # 2) Andar até o waypoint
            self.go_straight(distance)

            # Atualiza posição estimada
            self.x = wx
            self.y = wy
            self.theta = target_theta

        self.get_logger().info("🚀 Navegação concluída!")

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
