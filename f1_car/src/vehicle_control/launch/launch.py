from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os
def generate_launch_description():
    config = os.path.join(get_package_share_directory('param_server'), 'param.yaml')
    print(config)
    return LaunchDescription([
        Node(
            package='vehicle_control',
            namespace='',
            executable='control',
            parameters= [config],
            output = "screen",
            emulate_tty = True, #Comment this and the one before to get rid of the console prints
        )
    ])