import numpy as np
import rclpy
from scipy.interpolate import splev
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import math
from std_srvs.srv import SetBool
from trajectory_msg.srv import Trajectory, Feedback
#rom rclpy.action import ActionServer
from rclpy.node import Node
##remove this imports:
import matplotlib.pyplot as plt

class BaseController(Node):
    def __init__(self, FREQUENCY, ):
        self.dt = 1.0/FREQUENCY
        super().__init__('controller_node')
        self.state_subscriber = self.create_subscription( VehicleStateStamped, 'state',self.state_callback, 1)
        self.enabled = False
        self.vehicle_id = "car_id_not_set"
        self.pub = self.create_publisher( InputValues,'control', 1)
        self.controller_manager = self.create_service(SetBool, "controller_switch", self.controller_manager_callback)
        """
        self.trajectory_server = ActionServer(self, Trajectory,'execute_trajectory', self.execute_trajectory )
        self.action_feedback = Trajectory.Feedback()
        self.action_result = Trajectory.Result()
        self.current_state = VehicleStateStamped()
        """
        self.trajectory_srv = self.create_service(Trajectory, "execute_trajectory", self.execute_trajectory)
        self.feedback_client = self.create_client(Feedback, "vehicle_feedback")
        ##TODO: Have to create the client to send the progress of the car car and wether the execution is successful 
    def spin(self):
        rclpy.spin(self)
    def controller_manager_callback(self, request, response):
        if request.data == True:
            self.enabled = True
            response.success = True
            response.message = "Controller turned ON"
            print("Controller turned ON")
        else:
            self.enabled = False
            response.success = True
            response.message = "Controller turned OFF"
            print("Controller turned OFF")
        return response

    def state_callback(self, data):
        self.current_state = data
    def check_progress(self):
        """
        Function that checks if the execution of the trajectory has been successfull

        Returns:
            - True: The trajectory has been executed successfully.
            - False: The trajectory has not been executed successfully.
        """
        print(self.s, "/", self.s_end)
        feedback_req = Feedback.Request()
        feedback_req.progress = self.s / self.s_end*100
        feedback_req.succeeded = False
        feedback_req.car_id = self.vehicle_id #TODO !!!!!
        if abs(self.s-self.s_end)<0.01: # 5 cm deviation is enabled TODO: check this value in practice
            print("Goal achieved")
            feedback_req.succeeded = True
            self.enabled=False
            # stop the vehicle
            msg=InputValues()
            msg.d=0.0
            msg.delta=0.0
            self.pub.publish(msg)
            self.future = self.feedback_client.call_async(feedback_req) #We don't check wether the client got the progress #TODO: fix this???
            return True
        self.future = self.feedback_client.call_async(feedback_req) #We don't check wether the client got the progress #TODO: fix this???
        return False

    def execute_trajectory(self, trajectory_request, response):
        print("ez szar")
        path_t=np.array(trajectory_request.path_t)
        path_cx=np.array(trajectory_request.path_cx)
        path_cy=np.array(trajectory_request.path_cy)
        path_k=trajectory_request.path_k
        self.trajectory_tck=(path_t,[path_cx,path_cy],path_k)
        # set reference speed profile
        speed_t=np.array(trajectory_request.speed_t)
        speed_c=np.array(trajectory_request.speed_c)
        speed_k=trajectory_request.speed_k
        self.speed_tck=(speed_t,speed_c,speed_k)

        #set trajectory lenght
        self.s=trajectory_request.s_start 
        self.s_start=trajectory_request.s_start
        self.s_end=trajectory_request.s_end
        self.s_ref=self.s_start


        # check if the trajectory starts at current position
        pos, s0, z0, v, c=self.get_path_data(self.s_start)
        
        #current_state=rospy.wait_for_message("state",VehicleStateStamped)
        current_pos=np.array([self.current_state.position_x,self.current_state.position_y])
        theta_p=np.arctan2(s0[1], s0[0])
        
        # if reversing motion is required invert heading
        if v<0:
            current_heading=self.current_state.heading_angle+np.pi
        else:
            current_heading=self.current_state.heading_angle

        if abs(np.dot(current_pos-pos, z0))>0.5 or abs(_normalize(theta_p-_normalize(current_heading)))>0.5:
            self.get_logger().info("current_pos: {0}, {1}".format(current_pos[0],current_pos[1]))
            self.get_logger().info("ref_pos: {0}, {1}".format(pos[0],pos[1]))
            self.get_logger().info("heading: {0}".format(current_heading))
            self.get_logger().info("ref heading: {0}".format(theta_p))
            self.get_logger().info("lat_error: {0},  heading_error: {1}".format(abs(np.dot(current_pos-pos, z0)),abs(_normalize(theta_p-_normalize(current_heading)))))
        
       
        # enable state callbacks that trigger the control
        self.enabled=True
        print("Controller enabled")
        #When inheriting you have to return the response for the request


    def get_path(self,s):
        """
        Returns the path position at parameter s

        Arguments:
            - s(float): Path parameter/arc length
        """

        (x, y) = splev(s, self.trajectory_tck)
        return np.array([x,y])


    def get_path_data(self, s):
        """
        Helper function that returns path information at the specified parameter

        Arguments:
            - s(float): Parameter(arc length) of the Spline representing the path
        """
        # position & derivatives
        (x, y) = splev(s, self.trajectory_tck)
        (x_, y_) = splev(s, self.trajectory_tck, der=1)
        (x__,y__)=splev(s, self.trajectory_tck,der=2)
        
        # calculate base vectors of the moving coordinate frame
        s0 = np.array(
            [x_ / np.sqrt(x_**2 + y_**2), y_ / np.sqrt(x_**2 + y_**2)]
        )
        z0 = np.array(
            [-y_ / np.sqrt(x_**2 + y_**2), x_ / np.sqrt(x_**2 + y_**2)]
        )

        # calculate path curvature
        c=abs(x__*y_-x_*y__)/((x_**2+y_**2)**(3/2))

        # get speed reference
        v = splev(s, self.speed_tck)

        return np.array([x, y]), s0, z0, v, c

        
    def shutdown(self):
        """Stops the execution"""
        if self.enabled:
            self.enabled=False
            #self.action_result.success=False
            self.trajectory_srv.destroy()

    
def _clamp(value, bound):
    """
    Helper function that clamps the given value with the specified bounds

    Arguments:
        - value(float): The value to clamp
        - bound(tuple/int/float): If int/float the function constrains the value into [-bound,bound]
                                  If tuple the value is constained into the range of [bound[0],bound[1]]
    """
    if isinstance(bound, int) or isinstance(bound, float):
        if value < -bound:
            return -bound
        elif value > bound:
            return bound
        return value
    elif isinstance(bound, tuple):
        if value < bound[0]:
            return bound[0]
        elif value > bound[1]:
            return bound[1]
        return value

def project_to_closest(pos, s_est, s_window, path, step, s_bounds):
    """
    Projects the current vehicle position onto the closest position of the path

    Arguments:
        - pos(ndarray): x,y position of the vehicle
        - s_est(float): the estimated s arc length parameter of the path
        - window_s(float): The lenth of thew projection window
        - path(float): Function that returns the x,y position of the path at s
        - step(float): Step size between the points of evaluation in the projection window
        - s_bounds(tuple): The lower (s_bound[0]) and upper bound (s_bound[1]) of s
    """

    floor = _clamp(s_est - s_window / 2, s_bounds)
    ceil = _clamp(s_est + s_window, s_bounds)
    window = np.linspace(floor, ceil, round((ceil - floor) / step))
    path_points = path(window).T

    deltas = path_points - pos
    indx = np.argmin(np.einsum("ij,ij->i", deltas, deltas))
    return floor + indx * step


def _normalize(angle):
    """
    Normalizes the given angle into the [-pi/2, pi/2] range

    Arguments:
        - angle(float): The angle to normalize, in radian
    """
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi

    return angle