import rclpy
from rclpy.node import Node
import motioncapture
from mocap_msg.msg import PosValue ##Ez ki fog jönni,mert másik msg type lesz alkalmazva
from geometry_msgs.msg import PoseStamped
#from std_msgs.msg import Float64

class ParamServer(Node):


    def __init__(self):
        super().__init__("parameter_server")

        self.declare_parameters(
        namespace='',
        parameters=[
            ('vehicle_id_list', rclpy.Parameter.Type.STRING_ARRAY),
            ('hostname', rclpy.Parameter.Type.STRING),
            ('mocap_type', rclpy.Parameter.Type.STRING)
        ]
    )



class PositionPublisher(Node):
    def __init__(self, name):

        super().__init__( name+ "_pos") ##The node and the topic will have the same name
        self.publisher = self.create_publisher(PoseStamped, "/aimotion_mocap_node/rigid_bodies/"+name+ "/pose",1)
def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw ##copied from stackoverflow

def main():
    rclpy.init()
    print('Hi from mocap_pkg.')
    ps = ParamServer()
    hostname = ps.get_parameter('hostname').value
    vehicle_id_list = ps.get_parameter('vehicle_id_list').value
    mocap_type = ps.get_parameter('mocap_type').value
    publishers = {}
    print(vehicle_id_list)
    for name in vehicle_id_list:
        publishers[name] = PositionPublisher(name)
    print("publishers created...")
    print("Trying to connect to: ", hostname)
    mc = motioncapture.connect(mocap_type, {'hostname': hostname})
    print("Connected to: ",mocap_type)
    while True:
        mc.waitForNextFrame()
        for name, obj in mc.rigidBodies.items():
            if name in publishers.keys():
                msg = PoseStamped()
               
                msg.header.stamp = publishers[name].get_clock().now().to_msg()
                msg.pose.position.x = float(obj.position[0])
                msg.pose.position.y = float(obj.position[1])
                msg.pose.position.z = float(obj.position[2])
                msg.pose.orientation.x = float(obj.rotation.x)
                msg.pose.orientation.y = float(obj.rotation.y)
                msg.pose.orientation.z = float(obj.rotation.z)
                msg.pose.orientation.w = float(obj.rotation.w)
                publishers[name].publisher.publish(msg)
                


if __name__ == '__main__':
    main()
