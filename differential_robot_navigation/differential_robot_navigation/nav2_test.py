import rclpy # type: ignore
from nav2_simple_commander.robot_navigator import BasicNavigator # type: ignore
from geometry_msgs.msg import PoseStamped # type: ignore
import tf_transformations # type: ignore
import numpy as np # type: ignore

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
    nav = BasicNavigator()

    # --- Set initial pose
    initial_pose = create_pose_stamped(nav, 0., 0., 0.)

    nav.setInitialPose(initial_pose)

    # --- Wait for Nav2
    nav.waitUntilNav2Active()

    goal_pose = create_pose_stamped(nav, 3.5, -1.5, 135.0)
    nav.goToPose(goal_pose)

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        nav.get_logger().info(f"x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y}, Orientation: {feedback.current_pose.pose.orientation.z}, Distance remaining: {feedback.distance_remaining:.2f} m")

    result = nav.getResult()  # Get the result of the navigation task
    nav.get_logger().info(f"Navigation result: {result}")

    # --- Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()

