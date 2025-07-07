import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.br = CvBridge()
        self.get_logger().info('Nodo Procesador de Imágenes iniciado.')

    def listener_callback(self, data):
        # self.get_logger().info('Recibiendo fotograma de video') # Descomenta para depurar
        current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        
        # --- Lógica de Procesamiento de Imagen ---
        gray_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        
        # Muestra el fotograma procesado
        cv2.imshow("Video en Escala de Grises", gray_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()