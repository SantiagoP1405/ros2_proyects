from launch import LaunchDescription
from launch_ros.actions import Node # type: ignore
from launch.actions import IncludeLaunchDescription #type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch_ros.descriptions import ParameterValue # type: ignore
from launch.substitutions import Command # type: ignore
import os
from ament_index_python.packages import get_package_share_path, get_package_share_directory # type: ignore

def generate_launch_description():
    gz_launch_path = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch')
    
    urdf_path = os.path.join(get_package_share_path('differential_robot_description'),
        'urdf',
        'diff_segismundo.urdf.xacro'
    )

    rviz_path = os.path.join(get_package_share_path('differential_robot_description'),
        'rviz',
        'config.rviz'
    )

    gz_bridge_config_path = os.path.join(get_package_share_path('differential_robot_bringup'),
        'config',
        'gz_bridge.yaml')
    
    gz_world_path = os.path.join(get_package_share_path('differential_robot_bringup'),
        'worlds',
        'first_world.sdf')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}],
    )

    gz_sim_launch_path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_launch_path,
            "/gz_sim.launch.py"
        ]), launch_arguments={'gz_args' : f'{gz_world_path} -r'}.items()
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', '/robot_description']
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_path],
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file' : gz_bridge_config_path}]
    )
    return LaunchDescription([
        robot_state_publisher_node,
        gz_sim_launch_path,
        spawn_robot_node,
        rviz2_node,
        gz_bridge_node
    ])