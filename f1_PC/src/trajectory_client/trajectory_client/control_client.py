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
from ament_index_python.packages import get_package_share_directory
import os
traj_config = os.path.join(get_package_share_directory('trajectory_client'),  'traj_points.npy')
fig, ax = plt.subplots(2)
X_real = []
Y_real = []
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
        x = np.array(par)
        y = np.ones(len(par))*0.8
        step = 0/len(par)
        for i in range(len(par)):
            y[i] += step*i
        ax[0].plot(x,y, "o")
        #print(y)
        #ax[0].plot(self.x_points, self.y_points, "o")
# Generate the spline representation
        self.speed_tck = splrep(x, y, k=1, s=0.1)  # k is the degree, s is the smoothing factor (0 for interpolation)
# Evaluate the spline at the desired x-values
        
        self_speed_y = splev(self.u, self.speed_tck)
        ax[0].plot(par, self_speed_y)
        ax[0].relim()
        ax[0].set_title("Speed")
        ax[1].set_title("Trajectory")
        (self.X_ev,self.Y_ev)=splev(self.u, self.tck)
        ax[1].plot(self.X_ev,self.Y_ev)
        #print(par[-1])
        print("close plot window to send trajectory...")
        plt.show()
        

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
#path=Path(np.load("/home/bodlaire/Desktop/traj_points.npy"))
path=Path(np.load(traj_config))
class trajectory_client(Node):
    def __init__(self):
        super().__init__('trajectory_client')
        self.trajectory_client = self.create_client(Trajectory, '/execute_trajectory')
        self.progress_srv = self.create_service(Feedback, "vehicle_feedback",self.feedback_callback )
        msg_goal = Trajectory.Request()
        
        msg_goal.path_t = []
        for i in range(len(path.tck[0])):
            msg_goal.path_t.append(float(path.tck[0][i]))
        msg_goal.path_cx = []
        for i in range(len(path.tck[1][0])):
            msg_goal.path_cx.append(float(path.tck[1][0][i]))
        msg_goal.path_cy = []
        for i in range(len(path.tck[1][1])):
            msg_goal.path_cy.append(float(path.tck[1][1][i]))
        msg_goal.path_k = path.tck[2]


        msg_goal.speed_t = []
        #print(path.speed_tck)
        for i in range(len(path.speed_tck[0])):
            msg_goal.speed_t.append(float(path.speed_tck[0][i]))
        msg_goal.speed_c = []
        for i in range(len(path.speed_tck[1])):
            msg_goal.speed_c.append(float(path.speed_tck[1][i]))
        msg_goal.speed_k = path.speed_tck[2]
        msg_goal.s_start = 0.0
        msg_goal.s_end = float(path.length)
        print("sending goal...")
        self.send_request(msg_goal)

    def send_request(self, request):
        self.trajectory_client.wait_for_service()
        self.future = self.trajectory_client.call_async(request)
        print("Goal sent")
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result().received)


    def feedback_callback(self, request, response):
        print(request.car_id, " ", request.progress, " ", request.succeeded)
        if request.succeeded:
            print(request.car_id, " has finished trajectory")
            ax[1].plot(X_real, Y_real, "g")
            plt.show()
        response.received = True
        
        return response
def main():
    rclpy.init()
    t_client = trajectory_client() 
    rclpy.spin(t_client)


if __name__ == '__main__':
    main()
