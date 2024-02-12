import logging
import time
from threading import Timer

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

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
import datetime
from .submodules.radio_receiver import RadioReciever
import threading
class LoaderNode(Node):
    def __init__(self):
        super().__init__("parameter_server")
        self.declare_parameters(
            namespace= "",
            parameters=[
                ('car_id', rclpy.Parameter.Type.STRING),
                ("crazy_observer.FREQUENCY", rclpy.Parameter.Type.DOUBLE),
                ('crazy_observer.CUTOFF', rclpy.Parameter.Type.DOUBLE),
                ('crazy_observer.MARKER_OFFSET', rclpy.Parameter.Type.DOUBLE),
            ]
        )
         
class CrazyObserver(Node):
    def get_param(self):
        loader = LoaderNode()
        self.car_id = loader.get_parameter("car_id").value
        self.frequency = loader.get_parameter("crazy_observer.FREQUENCY").value
        self.cutoff = loader.get_parameter("crazy_observer.CUTOFF").value
        self.l_offs = loader.get_parameter("crazy_observer.MARKER_OFFSET").value
        self.dt=1.0/self.frequency

    def __init__(self, link_uri, filter_window):
        self.get_param()
        super().__init__(self.car_id+"_crazy_vehicle_state_observer")
        self.radio_receiver = RadioReciever(devid = 0)

        self.N=filter_window
        

        self.alpha = self.dt/(1.0/(self.cutoff*2*math.pi)+self.dt)


            # store previous timestamp
        self.prev_time=0.0

            # marker offset

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

        self.state_pub = self.create_publisher(VehicleStateStamped, self.car_id+'_state', 1)
        
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
        
    def spin(self):
        try:
            while True:
                self.process()
        except Exception as error:
            self.get_logger().error(f"{error}")
            self.radio_receiver.close()
            return
    def process(self):
        # get timestamp
        #time_str=str(data.header.stamp.sec)+"."+str(data.header.stamp.nanosec).zfill(9)
        buffer = self.radio_receiver.receive()
        if buffer is None:
            return
        time_float=float(buffer[0])
       # print(time_float)
        #time_float = Time.from_msg(data.header.stamp).nanoseconds /10**9 #??????????????????????????????? Is it working properly??
        # downsapling
        dt_=time_float-self.prev_time
        if abs(dt_)<self.dt*0.98: # only update data puffers
            return
        #print(dt_)
        # extract quaternion
        
      
        # calculate heading angle from quaternion
       

        #euler = self.euler_from_quaternion(data.pose.orientation)
        fi = buffer[1] ##equels to yaw
        if fi<0:
            fi=fi+2*math.pi
        self.condFi(fi)

        # extract position in OptiTrack's global coordinates
        # use l_offs and heading angle to calculate CoM
        x = buffer[2]-self.l_offs*np.cos(fi)
        y = buffer[3]-self.l_offs*np.sin(fi)
        #x=data.pose.position.x
        #y=data.pose.position.y

        # get previous and current point w/ moving average filter
        prev_x=self.getX()
        prev_y=self.getY()
        prev_fi=self.getFi()

        x=self.newX(x)
        y=self.newY(y)
        fi=self.newFi(fi)
        dfi = fi-prev_fi
        
        # relative position from previous point
        x = x - prev_x
        y = y - prev_y
       
        # heading angle difference
       

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
        """
        # rotate original point by heading
        """
        prev_x_rot = math.cos(-prev_fi)*prev_x - math.sin(-prev_fi)*prev_y
        prev_y_rot = math.sin(-prev_fi)*prev_x + math.cos(-prev_fi)*prev_y
        
        # obtain veloctiy by differentiation
        vx = (x_rot2 - prev_x_rot)/dt_
        vy = (y_rot2 - prev_y_rot)/dt_
       
        """
        # obtain veloctiy by crazyflie
        self.vx = data["stateEstimate.vx"]
        self.vy = data["stateEstimate.vy"]

        #vx_filt=vx
        #vy_filt=vy
        """
        self.vx_filt = self.vx_filt + (self.alpha*(vx - self.vx_filt))
        self.vy_filt = self.vy_filt + (self.alpha*(vy - self.vy_filt))
        
        #self.vx_filt=(x-prev_x)/dt_
        #self.vy_filt=(y-prev_y)/dt_
        """
        print("location delta:")
        print(x-prev_x)
        print(y-prev_y)
        print("delta t")
        
        print(dt_)
        
        print("------------------")
        print("speed")
        print(self.vx_filt)
        print(self.vy_filt)
        print("-_____________-")
        """
        # approximate yaw rateIn this software you have to define the rigidbodies corresponding to the vehicles, with unique ID-s, that are specified in the ```f1_PC/src/mocap_pkg/config/param.yaml``` file.

        ##Using CrazyFlie calculated speeds instead of using the calculated versions
        


        omega = dfi/dt_
        self.omega_filt = self.omega_filt + (self.alpha*(omega-self.omega_filt))

        # construct message & publish
        msg=VehicleStateStamped()

        msg.header=Header() #timestamp
        
        
        msg.header.stamp =  self.get_clock().now().to_msg()

        
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

        self.state_pub.publish(msg)  
# publish message

        # store timestamp
        self.prev_time=time_float

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
        
def main():
    uri = uri_helper.uri_from_env(default='usb://0')
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()
    rclpy.init()
    le = CrazyObserver(uri, filter_window = 3)
    le.get_logger().info("State observer node initialized!")
    t = threading.Thread(target=le.spin)
    t.start()
    t.join()
    le.radio_receiver.close()
    le.get_logger().info("Observer node shutting down!")


if __name__ == '__main__':
    main()
