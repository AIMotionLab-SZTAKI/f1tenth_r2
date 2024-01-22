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


class Path:
    def __init__(
        self,
        path_points,
    ):
        self.x_points = path_points[:, 0].tolist()
        self.y_points = path_points[:, 1].tolist() ###!!!!!!!!!!!!!!

        if (
            path_points[0, 0] == path_points[-1, 0]
            and path_points[0, 1] == path_points[-1, 1]
        ):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # closed s-t vissza 0.001-re
        elif len(self.x_points):
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # line
        else:
            tck, *rest = splprep([self.x_points, self.y_points], k=3, s=0.1)  # curve

        u = np.arange(0, 1.001, 0.001)
        path = splev(u, tck)

        (X, Y) = path
        s = np.cumsum(np.sqrt(np.sum(np.diff(np.array((X, Y)), axis=1) ** 2, axis=0)))
        self.length = s[-1]

        par = np.linspace(0, self.length, 1001)
        par = np.reshape(par, par.size)
        self.tck, self.u, *rest = splprep([X, Y], k=2, s=0.1, u= par)
        lines.set_xdata(numpy.array(X))
        lines.set_ydata(numpy.array(Y))
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()      
        print("Ref path initialised")
# Generate the spline representation
 
# Evaluate the spline at the desired x-values
        
        
        #plt.show()
        

"""
        [0, 0],
        [1, 1],
        [2, 2],
        [3, 2],
        [4, 1],
        [4.5, 0],
        [4, -1],
        [3, -2],
        [2, -2],
        [1, -1],
        [0, 0],
        [-1, 1],
        [-2, 2],
        [-3, 2],
        [-4, 1],
        [-4.5, 0],
        [-4, -2],
        [-3, -2],
        [-2, -2],
        [-1, -1],
        [0, 0],
    """
class state_client(Node):
    def __init__(self):
        super().__init__("state_check")
       
        self.sub = self.create_subscription( VehicleStateStamped, 'state',self.state_callback, 1)
    def state_callback(self,data):
        """
        if input("Save?\n")=="Y":
            plt.savefig('foo.png')
        """
        trajectory.set_xdata(numpy.append(trajectory.get_xdata(), data.position_x))
        trajectory.set_ydata(numpy.append(trajectory.get_ydata(), data.position_y))
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()  
        print("x:", data.position_x, " y:", data.position_y)
        
def main():
    rclpy.init()
    #path=Path(np.load("/home/bodlaire/Desktop/traj_points.npy"))
    plt.draw()
    s_c = state_client()
    rclpy.spin(s_c)
    

if __name__ == '__main__':
    main()
