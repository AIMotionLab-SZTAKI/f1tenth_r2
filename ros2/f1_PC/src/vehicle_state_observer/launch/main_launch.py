from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    state_config = os.path.join(get_package_share_directory('vehicle_state_observer'),  'param.yaml')
    mocap_config = os.path.join(get_package_share_directory('mocap_pkg'),  'param.yaml')

    return LaunchDescription([
        Node(
        package = "vehicle_state_observer",
        executable = "optitrack_state_observer_node",
        output = "screen",
        emulate_tty = True, #Comment this and the one before to get rid of the console prints
        parameters = [state_config],
        prefix = "xterm -e"
        ),
        Node(
        package = "mocap_pkg",
        executable = "mocap_node",
        output = "screen",
        emulate_tty = True, #Comment this and the one before to get rid of the console prints
        parameters = [mocap_config],
        prefix = "xterm -e"
        ),

    ])
