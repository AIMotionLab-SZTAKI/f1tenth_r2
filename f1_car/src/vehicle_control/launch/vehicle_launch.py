from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
def generate_launch_description():
    state_config = os.path.join(get_package_share_directory('vehicle_state_observer'),  'param.yaml')
    control_config = os.path.join(get_package_share_directory('vehicle_control'), 'param.yaml')
    vesc_config = os.path.join(get_package_share_directory('vesc_driver'),'params','vesc_config.yaml')
    drivebridge_config = os.path.join(get_package_share_directory('drive_bridge'), 'param.yaml')
    return LaunchDescription([
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
        Node(
            package='vehicle_control',
            namespace='',
            executable='control',
            parameters= [control_config],
            output = "screen",
            emulate_tty = True, #Comment this and the one before to get rid of the console prints
            #prefix = "xterm -e"
        ),
        Node(
            package = "vehicle_state_observer",
            executable = "optitrack_state_observer_node",
            output = "screen",
            emulate_tty = True, #Comment this and the one before to get rid of the console prints
            parameters = [state_config],
            #prefix = "xterm -e"
        ),
        Node(
            package='drive_bridge',
            namespace='',
            executable='drive_bridge',
            parameters= [drivebridge_config]
        )
    ])
