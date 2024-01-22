import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Header
from vesc_msgs.msg import VescStateStamped
from vehicle_state_msgs.msg import VehicleStateStamped
import math
import os
from rclpy.time import Time

class state_observer(Node):
#The idea is to implement the Estimator class from the previous build
#Reason: the node and the publish creation must be in the same class
#rospy <->rclpy :)
    def estimator_init_(self, l_offs,filter_window, frequency, cutoff):
            # moving average filter window
        self.N=filter_window

            # Step size
        self.dt=1.0/frequency

            # lowpass filter for output
        self.alpha = self.dt/(1.0/(cutoff*2*math.pi)+self.dt)


            # store previous timestamp
        self.prev_time=0.0

            # marker offset
        self.l_offs=l_offs

            # X coordinate
        self.x_list=[0 for _ in range(filter_window)]

            # Y coordinate
        self.y_list=[0 for _ in range(filter_window)]

            # Heading
        self.fi_list=[0 for _ in range(filter_window)]

            # Velocity
        self.vx_filt=0
        self.vy_filt=0

        self.omega_filt=0


            # VESC callback memory
        self.d=0.0
        self.delta=0.0
        self.ERPM=0.0

            # create rospy Publisher
            #self.state_pub=rospy.Publisher("state", VehicleStateStamped, queue_size=1)
        self.state_pub = self.create_publisher(VehicleStateStamped, 'state', 1)
        print('estimator initialized...')
    def euler_from_quaternion(self, quaternion):
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw #instead of using tf (as in the previous build)

    def newX(self,x):
        self.x_list.pop(0)
        self.x_list.append(x)

        return sum(self.x_list)/self.N

    def newY(self,y):
        self.y_list.pop(0)
        self.y_list.append(y)

        return sum(self.y_list)/self.N

    def condFi(self, newFi):
        if newFi-self.fi_list[-1]<-1.8*math.pi:
            self.fi_list=[item-2*math.pi for item in self.fi_list]
        elif newFi-self.fi_list[-1]>1.8*math.pi:
            self.fi_list=[item+2*math.pi for item in self.fi_list]



    def newFi(self,fi):
        if abs(fi-self.fi_list[-1])>1.8*math.pi:
            self.fi_list=[fi for _ in range(self.N)]
        else:
            self.fi_list.pop(0)
            self.fi_list.append(fi)

        return sum(self.fi_list)/self.N

    def getX(self):
        return sum(self.x_list)/self.N

    def getY(self):
        return sum(self.y_list)/self.N

    def getFi(self):
        return sum(self.fi_list)/self.N

    def setD(self,data):
        self.d=data.state.duty_cycle

    def setDelta(self, data):
        self.delta=data.data

    def setERPM(self, data):
        self.ERPM=data.state.speed

    def setData(self, data):
        self.setD(data)
        self.setERPM(data)

    def process(self,data):
        # get timestamp
        time_str=str(data.header.stamp.sec)+"."+str(data.header.stamp.nanosec).zfill(9)
        time_float=float(time_str)
        #time_float = Time.from_msg(data.header.stamp).nanoseconds /10**9 #??????????????????????????????? Is it working properly??
        # downsapling
        dt_=time_float-self.prev_time
        if abs(dt_)<self.dt*0.98: # only update data puffers
            return

        # extract quaternion
        
        quaternion = np.array([data.pose.orientation.x,
                           data.pose.orientation.y,
                           data.pose.orientation.z,
                           data.pose.orientation.w])

        
        # calculate heading angle from quaternion
       

        #euler = self.euler_from_quaternion(data.pose.orientation)
        r, p, y = self.euler_from_quaternion(quaternion)
        fi = y
        if fi<0:
            fi=fi+2*math.pi
        self.condFi(fi)

        # extract position in OptiTrack's global coordinates
        # use l_offs and heading angle to calculate CoM
        x = data.pose.position.x-self.l_offs*np.cos(fi)
        y = data.pose.position.y-self.l_offs*np.sin(fi)
        #x=data.pose.position.x
        #y=data.pose.position.y

        # get previous and current point w/ moving average filter
        prev_x=self.getX()
        prev_y=self.getY()
        prev_fi=self.getFi()

        x=self.newX(x)
        y=self.newY(y)
        fi=self.newFi(fi)

        # relative position from previous point
        x = x - prev_x
        y = y - prev_y

        # heading angle difference
        dfi = fi-prev_fi

        # rotate current positions by heading angle difference to eliminate the effect of steering
        x_rot = math.cos(-dfi)*x - math.sin(-dfi)*y
        y_rot = math.sin(-dfi)*x + math.cos(-dfi)*y

        # back to the original coordinate system
        x_rot = x_rot + prev_x
        y_rot = y_rot + prev_y
        x = x + prev_x
        y = y + prev_y

        # rotate by original heading
        x_rot2 = math.cos(-prev_fi)*x_rot - math.sin(-prev_fi)*y_rot
        y_rot2 = math.sin(-prev_fi)*x_rot + math.cos(-prev_fi)*y_rot

        # rotate original point by heading
        prev_x_rot = math.cos(-prev_fi)*prev_x - math.sin(-prev_fi)*prev_y
        prev_y_rot = math.sin(-prev_fi)*prev_x + math.cos(-prev_fi)*prev_y

        # obtain veloctiy by differentiation
        vx = (x_rot2 - prev_x_rot)/dt_
        vy = (y_rot2 - prev_y_rot)/dt_

        #vx_filt=vx
        #vy_filt=vy

        self.vx_filt = self.vx_filt + (self.alpha*(vx - self.vx_filt))
        self.vy_filt = self.vy_filt + (self.alpha*(vy - self.vy_filt))
	    #self.vx_filt=(x-prev_x)/dt_
	    #self.vy_filt=(y-prev_y)/dt_
        # approximate yaw rateIn this software you have to define the rigidbodies corresponding to the vehicles, with unique ID-s, that are specified in the ```f1_PC/src/mocap_pkg/config/param.yaml``` file.

        omega = dfi/dt_
        self.omega_filt = self.omega_filt + (self.alpha*(omega-self.omega_filt))

        # construct message & publish
        msg=VehicleStateStamped()

        msg.header=Header() #timestamp
        msg.header.stamp=data.header.stamp

        msg.position_x=x #position level data
        msg.position_y=y
        msg.heading_angle=fi

        msg.velocity_x=self.vx_filt #velocity level data
        msg.velocity_y=-self.vy_filt # negate because of OptiTrack-model transition
        msg.heading_angle=fi
        msg.omega=self.omega_filt

        msg.duty_cycle= float(self.d)
        msg.delta=self.delta
        msg.erpm=self.ERPM

        self.state_pub.publish(msg) # publish message

        # store timestamp
        self.prev_time=time_float

    def __init__(self):
        super().__init__("state_observer_node")
        self.declare_parameters(
        namespace='',
        parameters = [
            ('FREQUENCY', rclpy.Parameter.Type.DOUBLE),
            ('POSE_TOPIC', rclpy.Parameter.Type.STRING),
            ('CUTOFF', rclpy.Parameter.Type.DOUBLE),
            ('MARKER_OFFSET', rclpy.Parameter.Type.DOUBLE)
        ]
        )
        self.frequency = self.get_parameter('FREQUENCY').value
        self.pose_topic = self.get_parameter('POSE_TOPIC').value
        self.cutoff = float(self.get_parameter('CUTOFF').value)
        self.l_offs = float(self.get_parameter('MARKER_OFFSET').value)
        #self.estimator = Estimator(l_offs = self.l_offs, filter_window = 3, frequency = self.frequency, cutoff = self.cutoff)
        self.estimator_init_(l_offs = self.l_offs, filter_window = 3, frequency = self.frequency, cutoff = self.cutoff)

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.process,
            1)
        self.core_sub = self.create_subscription(
        VescStateStamped,
        'sensors/core',
        self.setData,
        1
        )
        self.servo_sub = self.create_subscription(
        Float64,
        'sensors/servo_position_command',
        self.setDelta,
        1
        )
        print('State observer initialized')

def main():
    rclpy.init()
    print('Hi from vehicle_state_observer.')

    so = state_observer()
    frequency = so.get_parameter('FREQUENCY').value
    pose_topic = so.get_parameter('POSE_TOPIC').value
    cutoff = float(so.get_parameter('CUTOFF').value)
    l_offs = float(so.get_parameter('MARKER_OFFSET').value)
    rclpy.spin(so)

if __name__ == '__main__':
    main()
