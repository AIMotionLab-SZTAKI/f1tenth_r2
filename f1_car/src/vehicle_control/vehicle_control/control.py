import numpy as np
import rclpy
from scipy.interpolate import splev
from scipy.interpolate import splprep
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import math
from trajectory_msg.srv import Trajectory, Feedback
from rclpy.action import ActionServer
from rclpy.node import Node
import matplotlib.pyplot as plt
from .submodules.control_util import BaseController
from .submodules.combined_controller import CombinedController
class LoaderNode(Node):
    def __init__(self):
        super().__init__("parameter_server")

        
        self.declare_parameters(
            namespace= "",
            parameters=[
                ('car_id', rclpy.Parameter.Type.STRING),
                ("controller.FREQUENCY", rclpy.Parameter.Type.DOUBLE),
                ('controller.LATERAL_CONTROL_GAINS.k1',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LATERAL_CONTROL_GAINS.k2',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LATERAL_CONTROL_GAINS.k3',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LATERAL_CONTROL_GAINS.k1_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LATERAL_CONTROL_GAINS.k2_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LONGITUDINAL_CONTROL_GAINS.k1' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LONGITUDINAL_CONTROL_GAINS.k2' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controller.LONGITUDINAL_CONTROL_GAINS.m' ,rclpy.Parameter.Type.DOUBLE),
                ('controller.LONGITUDINAL_CONTROL_GAINS.C_m1' ,rclpy.Parameter.Type.DOUBLE),
                ('controller.LONGITUDINAL_CONTROL_GAINS.C_m2' ,rclpy.Parameter.Type.DOUBLE),
                ('controller.LONGITUDINAL_CONTROL_GAINS.C_m3' ,rclpy.Parameter.Type.DOUBLE),
                
            ]
        )
        
def main():
    print('Hi from vehicle_control!')
    rclpy.init()
    loader = LoaderNode()
    
    car_id = loader.get_parameter('car_id').value
    freq = loader.get_parameter('controller.FREQUENCY').value
    lat_gains = {
        'k1': loader.get_parameter("controller.LATERAL_CONTROL_GAINS.k1").value,
        'k2': loader.get_parameter("controller.LATERAL_CONTROL_GAINS.k2").value,
        'k3': loader.get_parameter("controller.LATERAL_CONTROL_GAINS.k3").value,
        'k1_r': loader.get_parameter("controller.LATERAL_CONTROL_GAINS.k1_r").value,
        'k2_r': loader.get_parameter("controller.LATERAL_CONTROL_GAINS.k2_r").value,
    }
    long_gains = {
        'k1': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.k1").value,
        'k2': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.k2").value,
        'm': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.m").value,
        'C_m1': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.C_m1").value,
        'C_m2': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.C_m2").value,
        'C_m3': loader.get_parameter("controller.LONGITUDINAL_CONTROL_GAINS.C_m3").value,
    }
    
    loader.destroy_node()
    controller=CombinedController(FREQUENCY=freq,projection_window=0.5, lateral_gains=lat_gains,longitudinal_gains=long_gains, projection_step=0.001, vehicle_id = car_id,look_ahead=0.04)
    controller.vehicle_id = car_id
    rclpy.spin(controller)

if __name__ == '__main__':
    main()
