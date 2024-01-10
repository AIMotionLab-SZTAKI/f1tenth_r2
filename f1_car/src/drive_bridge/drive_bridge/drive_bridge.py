import rclpy
from rclpy.node import Node
from drive_bridge_msg.msg import InputValues
from std_msgs.msg import Float64
import yaml
import os
class LoaderNode(Node):
    def __init__(self):
        super().__init__("parameter_server")

        
        self.declare_parameters(
            namespace= "",
            parameters=[
                ('car_id', rclpy.Parameter.Type.STRING),
                ('drive_bridge.STEERING_GAINS', rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('drive_bridge.MOTOR_LIMIT', rclpy.Parameter.Type.DOUBLE)
        
            ]
        )
        
class DriveBridge(Node):
    def __init__(self, car_id):
        super().__init__('drive_bridge')
        loader = LoaderNode()
       
        self.angle_offset=float(loader.get_parameter('drive_bridge.STEERING_GAINS').value[1])
        self.angle_gain=float(loader.get_parameter("drive_bridge.STEERING_GAINS").value[0])
        self.reference_limit=float(loader.get_parameter("drive_bridge.MOTOR_LIMIT").value)
        
        #Create Publishers
        self.delta_pub = self.create_publisher(Float64, "commands/servo/position",1)
        self.duty_pub = self.create_publisher(Float64, "commands/motor/duty_cycle",1)
        self.emergency_shutdown = False
    
    
    def clamp_d(self, d):
        if d<-self.reference_limit:
            d=-self.reference_limit
        elif d > self.reference_limit:
            d=self.reference_limit
        return d
    

    def send_commands(self, data):
        if not self.emergency_shutdown:
            steering_angle = self.angle_offset+self.angle_gain*data.delta
            print(data.delta*self.angle_gain , ' + ' , self.angle_offset)
            msg=Float64()
            msg.data=steering_angle
            self.delta_pub.publish(msg)
            msg=Float64()
            msg.data=self.clamp_d(data.d)
            self.duty_pub.publish(msg)
        else:
            # publish messages
            self.delta_pub.publish(self.angle_offset)
            self.duty_pub.publish(0)

    def shutdown(self):
        # publish messages
        self.delta_pub.publish(self.angle_offset)
        self.duty_pub.publish(0)


def main():
    print('Hi from drive_bridge.')
    rclpy.init()
    car_id = "JoeBushJr"
    drive_bridge = DriveBridge(car_id)
    drive_bridge.create_subscription(topic='control',msg_type= InputValues, callback= drive_bridge.send_commands,qos_profile=1)
    rclpy.spin(drive_bridge)
    drive_bridge.shutdown()
    drive_bridge.destroy_node()
    drive_bridge.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
