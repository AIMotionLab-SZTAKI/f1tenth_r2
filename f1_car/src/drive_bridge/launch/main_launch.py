from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
def generate_launch_description():
    drive_bridge_config = os.path.join(get_package_share_directory('drive_bridge'), 'param.yaml')
    vesc_config = os.path.join(
        get_package_share_directory('vesc_driver'),
        'params',
        'vesc_config.yaml'
        )
    return LaunchDescription([
        Node(
            package='drive_bridge',
            namespace='',
            executable='drive_bridge',
            parameters= [drive_bridge_config]
        ),
        DeclareLaunchArgument(
            name="config",
            default_value=vesc_config,
            description="VESC yaml configuration file.",
            ),
        Node(
            package='vesc_driver',
            executable='vesc_driver_node',
            name='vesc_driver_node',
            parameters=[LaunchConfiguration("config")]
        ),
    ])