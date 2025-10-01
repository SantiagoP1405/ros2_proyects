import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import cv2
import math

class LaneControlNode(Node):
    def __init__(self):
        super().__init__('lane_control_node')
        self.pub = self.create_publisher(String, '/lane/control', 10)
        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz
        self.last_cmd = ""  # 'A','S','C' o mapeo que decidas
        self.sub = self.create_subscription(String,
                                            '/signal/detection',
                                            self.cmd_cb,
                                            10)
        self.get_logger().info('LaneControlNode iniciado.')

    def cmd_cb(self, msg: String):
        # Guarda el último comando externo para priorizarlo una vez
        self.last_cmd = msg.data

    def loop(self):
        # 1) Lee frame
        ret, img_bgr = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo leer el fotograma.')
            return

        # 2) Pipeline de líneas
        img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        img_rgb  = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_blur = cv2.GaussianBlur(img_grey, (3, 3), 0, 0)
        img_canny = cv2.Canny(img_blur, 40, 120)

        vertices = np.array([[(0,480), (150,0), (450,0), (640,480)]], dtype=np.int32)
        img_roi = np.zeros_like(img_grey)
        cv2.fillPoly(img_roi, vertices, 255)
        img_mask = cv2.bitwise_and(img_canny, img_roi)

        rho, theta, threshold = 2, np.pi/180, 40
        min_line_len, max_line_gap = 50, 10
        lines = cv2.HoughLinesP(img_mask, rho, theta, threshold, np.array([]),
                                minLineLength=min_line_len, maxLineGap=max_line_gap)

        img_lines = np.zeros((img_mask.shape[0], img_mask.shape[1], 3), dtype=np.uint8)
        suma, contador = 0.0, 0
        suma_centros_x, lineas_validas = 0.0, 0

        if lines is not None:
            for line in lines:
                for x1,y1,x2,y2 in line:
                    centro_x = (x1 + x2) // 2
                    suma_centros_x += centro_x
                    lineas_validas += 1
                    dx = x2 - x1
                    if dx == 0:
                        continue
                    dy = y2 - y1
                    ang_deg = math.degrees(math.atan2(dy, dx))
                    if ang_deg < 0:
                        ang_deg += 180
                    cv2.line(img_lines, (x1,y1), (x2,y2), (255,0,0), 3)
                    contador += 1
                    suma += abs(ang_deg)

        promedio = (suma / contador) if contador > 0 else 90.0
        umbral_giro = 5.0
        ang_conduccion = 0.0
        direccion = "F"

        if abs(promedio - 90.0) < umbral_giro:
            ang_conduccion, direccion = 0.0, "F"
        elif promedio < 90.0:
            ang_conduccion = min(90.0 - promedio, 45.0)
            direccion = "L"
        else:
            ang_conduccion = min(promedio - 90.0, 45.0)
            direccion = "R"

        height, width = img_mask.shape
        zona_central_min = width // 2 - 50
        zona_central_max = width // 2 + 50
        esquina_izq, esquina_der = 165, 460

        if lineas_validas > 0:
            centro_prom_x = suma_centros_x / lineas_validas
            if centro_prom_x < esquina_izq:
                direccion, ang_conduccion = "I", 20.0
            elif centro_prom_x > esquina_der:
                direccion, ang_conduccion = "D", 20.0
            elif centro_prom_x < zona_central_min:
                direccion, ang_conduccion = "L", 6.0
            elif centro_prom_x > zona_central_max:
                direccion, ang_conduccion = "R", 6.0

        # 3) Prioridad a comando externo UNA VEZ si existe
        #    Mapea 'A','S','C' a tus direcciones si aplica
        if self.last_cmd:
            cmd = self.last_cmd
            self.last_cmd = ""
            # Ejemplo de mapeo: A=adelante, S=stop, C=cruce/centro
            if cmd == 'A':
                direccion = "A"; ang_conduccion = 0.0
            elif cmd == 'S':
                direccion = "S"; ang_conduccion = 0.0 
            elif cmd == 'C':
                direccion = "C"; ang_conduccion = 0.0

        angulo_redondeado = f"{int(math.floor(ang_conduccion)):02d}"
        mensaje = f"{angulo_redondeado}{direccion}"

        out = String()
        out.data = mensaje
        self.pub.publish(out)
        self.get_logger().info(f'Lane control: {mensaje}')

        # 4) Vista
        img_lane_lines = cv2.addWeighted(img_rgb, 1, img_lines, 1, 1)
        cv2.imshow('lane', img_lane_lines)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LaneControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
