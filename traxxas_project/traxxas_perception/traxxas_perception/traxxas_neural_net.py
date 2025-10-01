import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import tensorflow as tf
from ament_index_python.packages import get_package_share_directory
import os

class CameraSignalNode(Node):
    def __init__(self):
        super().__init__('camera_signal_node')
        self.publisher_ = self.create_publisher(String, '/signal/detection', 10)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # Obtener la ruta del paquete y construir la ruta al modelo
        package_path = get_package_share_directory('traxxas_perception')
        model_path = os.path.join(package_path, 'modelo_senales.tflite')
        
        self.interpreter = tf.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        timer_period = 0.05  # ~20 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Nodo combinado iniciado.')
        
    def timer_callback(self):
        ret, img_nn = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo leer el fotograma de la cámara.')
            return

        img_rgb_nn = cv2.cvtColor(img_nn, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb_nn, (224, 224))
        input_data = img_resized.astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        pred_clase = np.argmax(output)
        confianza = output[pred_clase]

        if confianza > 0.95:
            msg_out = String()
            if pred_clase == 0:
                msg_out.data = 'A'
            elif pred_clase == 2:
                msg_out.data = 'S'
            elif pred_clase == 1:
                msg_out.data = 'C'
            self.publisher_.publish(msg_out)
            self.get_logger().info(f'Detección: {msg_out.data} (Confianza: {confianza:.2f})')

        # Visualiza la imagen (opcional)
        cv2.imshow('Ventana', img_nn)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CameraSignalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
