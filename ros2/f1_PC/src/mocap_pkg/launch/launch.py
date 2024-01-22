from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
def generate_launch_description():
    config = os.path.join(get_package_share_directory('mocap_pkg'),  'param.yaml')
    ##print("Config file:",config) ##Uncomment this to see the location of the config file
    return LaunchDescription([
        Node(
        package = "mocap_pkg",
        executable = "mocap_node",
        output = "screen",
        emulate_tty = True, #Comment this and the one before to get rid of the console prints
        parameters = [config],
        ),
    ])
