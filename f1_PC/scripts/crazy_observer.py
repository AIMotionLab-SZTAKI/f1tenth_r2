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

uri = uri_helper.uri_from_env(default='usb://0')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


class CrazyObserver(Node):
    def process(self, timestamp, data, logconf):
        # get timestamp
        #time_str=str(data.header.stamp.sec)+"."+str(data.header.stamp.nanosec).zfill(9)
        time_float=float(timestamp)
        #time_float = Time.from_msg(data.header.stamp).nanoseconds /10**9 #??????????????????????????????? Is it working properly??
        # downsapling
        dt_=time_float-self.prev_time
        if abs(dt_)<self.dt*0.98: # only update data puffers
            return

        # extract quaternion
        
      
        # calculate heading angle from quaternion
       

        #euler = self.euler_from_quaternion(data.pose.orientation)
        r = data["stabilizer.roll"]
        p = data["stabilizer.pitch"]
        y = data["stabilizer.yaw"]
        fi = y
        if fi<0:
            fi=fi+2*math.pi
        self.condFi(fi)

        # extract position in OptiTrack's global coordinates
        # use l_offs and heading angle to calculate CoM
        x = data["stateEstimate.x"]-self.l_offs*np.cos(fi)
        y = data["stateEstimate.y"]-self.l_offs*np.sin(fi)
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

    def __init__(self, link_uri, l_offs,filter_window, frequency, cutoff):
        self._cf = Crazyflie(rw_cache='./cache')
        super().__init__("state_observer_node")
        # Connect some callbacks from the Crazyflie API
       
        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        self.is_connected = True
        self.N=filter_window
        self.dt=1.0/frequency

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

        self._cf.connected.add_callback(self._connected)
        #self._cf.disconnected.add_callback(self._disconnected)
        #self._cf.connection_failed.add_callback(self._connection_failed)
        #self._cf.connection_lost.add_callback(self._connection_lost)
        self.state_pub = self.create_publisher(VehicleStateStamped, 'state', 1)
        
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
        print('Crazy_Observer initialized...')

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        self._lg_stab.add_variable('stateEstimate.x', 'float')
        self._lg_stab.add_variable('stateEstimate.y', 'float')
        self._lg_stab.add_variable('stateEstimate.z', 'float')
        self._lg_stab.add_variable('stabilizer.roll', 'float')
        self._lg_stab.add_variable('stabilizer.pitch', 'float')
        self._lg_stab.add_variable('stabilizer.yaw', 'float')
        # The fetch-as argument can be set to FP16 to save space in the log packet
        self._lg_stab.add_variable('pm.vbat', 'FP16')

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self._cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self.process)
            # This callback will be called on errors
            #self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    rclpy.init()
    le = CrazyObserver(uri, l_offs= 0.0, filter_window= 3, frequency= 60.0, cutoff= 2.0)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.

    """
    while le.is_connected:
        time.sleep(1) ## Maybe this won't be used becaues of rclpy.spin()
    """
