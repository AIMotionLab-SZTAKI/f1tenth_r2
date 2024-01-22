from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(get_package_share_directory('param_server'), 'param.yaml')
    return LaunchDescription([
        Node(
            package='drive_bridge',
            namespace='',
            executable='drive_bridge',
            parameters= [config]
        )
    ])