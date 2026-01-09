import rclpy # type: ignore
from rclpy.node import Node  # type: ignore
from nav2_simple_commander.robot_navigator import BasicNavigator # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
from std_msgs.msg import String # type: ignore
import tf_transformations # type: ignore
import numpy as np # type: ignore
import time

class PublishPoint(Node):
    def __init__(self):
        super().__init__('publish_point_node')
        self.nav = BasicNavigator()
        
        # Publisher para el estado del robot
        self.status_publisher = self.create_publisher(
            String,
            '/robot_status',
            10
        )
        
        # Establecer posición inicial
        initial_pose = create_pose_stamped(self.nav, 0., -7., 90.)
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active()
        
        # Waypoints predefinidos para patrullaje
        self.waypoints = [
            {"name": "Pasillo_1_1", "x": 2.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_2", "x": 3.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_3", "x": 4.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_4", "x": 5.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_5", "x": 6.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_6", "x": 7.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_7", "x": 8.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_1_8", "x": 8.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_9", "x": 7.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_10", "x": 6.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_11", "x": 5.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_12", "x": 4.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_13", "x": 3.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_1_14", "x": 2.5, "y": -5.18, "yaw": np.pi},

            {"name": "Pasillo_Central_1", "x": 0.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_Central_2", "x": 0.5, "y": -2.85, "yaw": np.pi/2},

            {"name": "Pasillo_2_1", "x": 2.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_2", "x": 3.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_3", "x": 4.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_4", "x": 5.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_5", "x": 6.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_6", "x": 7.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_7", "x": 8.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_2_8", "x": 8.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_9", "x": 7.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_10", "x": 6.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_11", "x": 5.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_12", "x": 4.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_13", "x": 3.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_2_14", "x": 2.5, "y": -2.85, "yaw": np.pi},

            {"name": "Pasillo_Central_3", "x": 0.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_Central_4", "x": 0.5, "y": -0.56, "yaw": np.pi/2},

            {"name": "Pasillo_3_1", "x": 2.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_2", "x": 3.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_3", "x": 4.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_4", "x": 5.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_5", "x": 6.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_6", "x": 7.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_7", "x": 8.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_3_8", "x": 8.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_9", "x": 7.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_10", "x": 6.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_11", "x": 5.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_12", "x": 4.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_13", "x": 3.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_3_14", "x": 2.5, "y": -0.56, "yaw": np.pi},

            {"name": "Pasillo_Central_5", "x": 0.0, "y": -0.56, "yaw": np.pi},

            {"name": "Pasillo_4_1", "x": -2.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_2", "x": -3.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_3", "x": -4.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_4", "x": -5.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_5", "x": -6.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_6", "x": -7.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_7", "x": -8.5, "y": -0.56, "yaw": np.pi},
            {"name": "Pasillo_4_8", "x": -8.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_9", "x": -7.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_10", "x": -6.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_11", "x": -5.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_12", "x": -4.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_13", "x": -3.5, "y": -0.56, "yaw": 0.0},
            {"name": "Pasillo_4_14", "x": -2.5, "y": -0.56, "yaw": 0.0},
            
            {"name": "Pasillo_Central_6", "x": -0.5, "y": -0.56, "yaw": 0},
            {"name": "Pasillo_Central_7", "x": -0.5, "y": -2.85, "yaw": -np.pi/2},

            {"name": "Pasillo_5_1", "x": -2.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_2", "x": -3.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_3", "x": -4.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_4", "x": -5.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_5", "x": -6.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_6", "x": -7.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_7", "x": -8.5, "y": -2.85, "yaw": np.pi},
            {"name": "Pasillo_5_8", "x": -8.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_9", "x": -7.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_10", "x": -6.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_11", "x": -5.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_12", "x": -4.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_13", "x": -3.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_5_14", "x": -2.5, "y": -2.85, "yaw": 0.0},

            {"name": "Pasillo_Central_8", "x": -0.5, "y": -2.85, "yaw": 0.0},
            {"name": "Pasillo_Central_9", "x": -0.5, "y": -5.18, "yaw": -np.pi/2},

            {"name": "Pasillo_6_1", "x": -2.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_2", "x": -3.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_3", "x": -4.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_4", "x": -5.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_5", "x": -6.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_6", "x": -7.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_7", "x": -8.5, "y": -5.18, "yaw": np.pi},
            {"name": "Pasillo_6_8", "x": -8.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_9", "x": -7.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_10", "x": -6.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_11", "x": -5.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_12", "x": -4.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_13", "x": -3.5, "y": -5.18, "yaw": 0.0},
            {"name": "Pasillo_6_14", "x": -2.5, "y": -5.18, "yaw": 0.0},

            {"name": "Pasillo_Central_10", "x": 0.0, "y": -5.18, "yaw": 0.0},


        ]
        
        # Variables de estado
        self.current_waypoint_index = -1
        self.listening_time = 3.0  # segundos en cada punto
        self.current_zone = "Posición Inicial"
        
        # Timer para controlar el patrullaje
        self.patrol_timer = self.create_timer(1.0, self.patrol_state_machine)
        
        # Estados del robot
        self.state = "WAITING"  # WAITING, MOVING, LISTENING
        self.listen_start_time = None
        
        self.get_logger().info('🤖 Robot iniciado en posición inicial (0, -7, 90°)')
        self.get_logger().info(f'📍 {len(self.waypoints)} waypoints de patrullaje cargados')
        
    def start_patrol(self):
        """Iniciar el patrullaje automático"""
        self.get_logger().info('🔄 Iniciando patrullaje automático...')
        
        # Publicar inicio de patrullaje
        status_msg = String()
        status_msg.data = "PATRULLAJE_INICIADO"
        self.status_publisher.publish(status_msg)
        
        self.state = "MOVING"
        self.move_to_next_waypoint()
    
    def create_pose_from_waypoint(self, waypoint):
        """Crear PoseStamped desde un waypoint con orientación en radianes"""
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, waypoint["yaw"])
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = waypoint["x"]
        pose.pose.position.y = waypoint["y"]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q_x
        pose.pose.orientation.y = q_y
        pose.pose.orientation.z = q_z
        pose.pose.orientation.w = q_w
        
        return pose
    
    def move_to_next_waypoint(self):
        """Moverse al siguiente waypoint"""
        self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
        next_waypoint = self.waypoints[self.current_waypoint_index]
        
        self.get_logger().info(f'🚀 Navegando hacia {next_waypoint["name"]} ({next_waypoint["x"]:.2f}, {next_waypoint["y"]:.2f})')
        
        # Publicar próximo destino
        status_msg = String()
        status_msg.data = f"NAVEGANDO_A: {next_waypoint['name']}"
        self.status_publisher.publish(status_msg)
        
        pose = self.create_pose_from_waypoint(next_waypoint)
        self.nav.goToPose(pose)
        self.state = "MOVING"
    
    def patrol_state_machine(self):
        """Máquina de estados para el patrullaje"""
        
        if self.state == "WAITING":
            # Estado inicial, esperando comando para iniciar
            pass
            
        elif self.state == "MOVING":
            # Verificar si llegó al destino
            if self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                if feedback:
                    self.get_logger().info(
                        f"Posición actual: ({feedback.current_pose.pose.position.x:.2f}, "
                        f"{feedback.current_pose.pose.position.y:.2f}), "
                        f"Distancia restante: {feedback.distance_remaining:.2f} m"
                    )
                
                result = self.nav.getResult()
                current_waypoint = self.waypoints[self.current_waypoint_index]
                
                if result:
                    self.get_logger().info(f"✅ Llegué a {current_waypoint['name']}")
                    self.current_zone = current_waypoint['name']
                    self.state = "LISTENING"
                    self.listen_start_time = time.time()
                    
                    # Publicar que llegó a la zona
                    status_msg = String()
                    status_msg.data = f"ROBOT_EN_ZONA: {self.current_zone}"
                    self.status_publisher.publish(status_msg)
                    
                else:
                    self.get_logger().warn(f"❌ No pude llegar a {current_waypoint['name']}")
                    self.move_to_next_waypoint()
        
        elif self.state == "LISTENING":
            # Permanecer en el punto por el tiempo especificado
            elapsed_time = time.time() - self.listen_start_time
            remaining_time = self.listening_time - elapsed_time
            
            if remaining_time > 0:
                if int(remaining_time) % 3 == 0:  # Log cada 3 segundos
                    self.get_logger().info(f"⏱️ En {self.current_zone} - {remaining_time:.0f}s restantes")
                    
                    # Publicar estado
                    status_msg = String()
                    status_msg.data = f"ESPERANDO_EN: {self.current_zone} - {remaining_time:.0f}s restantes"
                    self.status_publisher.publish(status_msg)
                    
            else:
                self.get_logger().info(f"✅ Tiempo completado en {self.current_zone}")
                self.move_to_next_waypoint()

def create_pose_stamped(navigator:BasicNavigator, x:float, y:float, q_z:float) -> PoseStamped:
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(q_z)) # type: ignore
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose

def main():
    # --- Init
    rclpy.init()
    node = PublishPoint()
    
    # Esperar un momento y luego iniciar el patrullaje automático
    time.sleep(2)
    node.start_patrol()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()