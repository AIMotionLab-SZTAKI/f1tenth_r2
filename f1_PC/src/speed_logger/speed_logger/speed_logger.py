import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msg.srv import Trajectory, Feedback
import numpy as np
import math
from scipy.interpolate import splev, splprep, splrep
import scipy.interpolate
import numpy
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, Float64
from vehicle_state_msgs.msg import VehicleStateStamped
plt.ion()
fig, ax = plt.subplots()
X_real = []
Y_real = []
lines, = ax.plot([],[], "g")
trajectory, = ax.plot([],[], "r")
        #Autoscale on unknown axis and known lims on the other
ax.set_autoscaley_on(True)
ax.grid()



class state_client(Node):
    def __init__(self):
        super().__init__("state_check")
       
        self.sub = self.create_subscription( VehicleStateStamped, 'state',self.state_callback, 1)
        self.i = 0
    def state_callback(self,data):
        """
        if input("Save?\n")=="Y":
            plt.savefig('foo.png')
        """
        speed = pow(pow(data.velocity_x,2)+pow(data.velocity_y,2),1/2)
        trajectory.set_ydata(numpy.append(trajectory.get_ydata(), speed))
        trajectory.set_xdata(numpy.append(trajectory.get_xdata(), self.i ))
        self.i+=1
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()  
        print("x:", data.position_x, " y:", data.position_y)
        
def main():
    rclpy.init()
    plt.draw()
    s_c = state_client()
    rclpy.spin(s_c)
    

if __name__ == '__main__':
    main()
