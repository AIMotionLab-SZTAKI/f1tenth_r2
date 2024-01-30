from rclpy.node import Node
from drive_bridge_msg.msg import InputValues




class ControlPublisher(Node):
    def __init__(self, vehicle_name:str):
        super().__init__(vehicle_name+"_remote_contoller")
        self.command_pub = self.create_publisher(InputValues, "/" + vehicle_name + "_control",1)

    def publish(self, delta:float, duty_cycle: float):

        msg = InputValues()
        msg.delta = delta
        msg.d = duty_cycle
        self.command_pub.publish(msg)
