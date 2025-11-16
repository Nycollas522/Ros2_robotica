#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraFilterNode(Node):
    def __init__(self):
        super().__init__('camera_filter_node')
        self.bridge = CvBridge()
        
        # Subscrição ao tópico da câmera (verifique o nome exato no Gazebo!)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # ou /camera/color/image_raw dependendo do modelo
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Converte imagem ROS → OpenCV (BGR8 é o formato comum)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # --- APLIQUE SEU FILTRO AQUI ---
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 100, 200)

            # Mostra a imagem filtrada
            cv2.imshow("Imagem Filtrada", edges)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
