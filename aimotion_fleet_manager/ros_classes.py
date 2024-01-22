from trajectory_msg.srv import Trajectory
from vehicle_state_msgs.msg import VehicleStateStamped

from rclpy.node import Node
from PyQt5.QtCore import  QObject, QThread
import rclpy
import csv #used for logging

class logger_node_process(QThread):

    def __init__(self, vehicle_name):
        super(logger_node_process, self).__init__()
        self.node = trajectory_node(vehicle_name=vehicle_name) 
        ##TODO



class logger_node(Node):
    def __init__(self, vehicle_name):
        super().__init__(vehicle_name + "logger_node")
        self.create_subscription(
            msg_type=VehicleStateStamped,
            topic= vehicle_name+ "_state",
            callback= self.state_callback
            )
        

        self.logging_status = False
        
        self.fieldnames = ['time_stamp_sec', 'position_x', 'position_y', 'heading_angle', 'velocity_x', 'velocity_y', 'omega', 'duty_cycle', 'delta', 'erpm']
        
        self.csv_file =  open('my_log.csv', mode='w')
        
        self.writer = csv.DictWriter(self.csv_file, fieldnames= self.fieldnames)
        
        self.writer.writeheader()

    def state_callback(self, data):
        if self.logging_status == True: #implemented from the previous remote controller

            self.writer.writerow({
                'time_stamp_sec':data.header.stamp.sec + data.header.stamp.nanosec/10**9,
                'position_x': data.position_x,
                'position_y': data.position_y,
                'heading_angle': data.heading_angle,
                'velocity_x': data.velocity_x,
                'velocity_y': data.velocity_y,
                'omega': data.omega,
                'duty_cycle': data.duty_cycle,
                'delta': data.delta,
                'erpm': data.erpm
                  })




class trajectory_client_process(QThread):
    """
    Provides start() func to run async the rclpy.spin for a trajectory_node instance
    """


    def __init__(self, vehicle_name):

        super(trajectory_client_process, self).__init__()


        self.node = trajectory_node(vehicle_name=vehicle_name)


    def run(self):

        rclpy.spin(self.node)

class trajectory_node(Node):
    def __init__(self, vehicle_name):

        super().__init__(vehicle_name+'_trajectory_client')

        self.trajectory_client = self.create_client(Trajectory, vehicle_name+'_execute_trajectory')
        
        self.progress_srv =  None#self.create_service(Feedback, vehicle_name+"_vehicle_feedback",self.feedback_callback )
        
        
    def send_request(self, path):

        request = Trajectory.Request()
        
        request.path_t = []
        for i in range(len(path.tck[0])):
            request.path_t.append(float(path.tck[0][i]))
        request.path_cx = []
        for i in range(len(path.tck[1][0])):
            request.path_cx.append(float(path.tck[1][0][i]))
        request.path_cy = []
        for i in range(len(path.tck[1][1])):
            request.path_cy.append(float(path.tck[1][1][i]))
        request.path_k = path.tck[2]


        request.speed_t = []
        #print(path.speed_tck)
        for i in range(len(path.speed_tck[0])):
            request.speed_t.append(float(path.speed_tck[0][i]))
        request.speed_c = []
        for i in range(len(path.speed_tck[1])):
            request.speed_c.append(float(path.speed_tck[1][i]))
        request.speed_k = path.speed_tck[2]
        request.s_start = 0.0
        request.s_end = float(path.length)

        try:
            if self.trajectory_client.wait_for_service(timeout_sec= 2) == False:
                raise SystemError("Service is not running")
            self.future = self.trajectory_client.call_async(request)


        #rclpy.spin_until_future_complete(self, self.future)
        #print(self.future.result().received)
        except Exception as error :
            print(error)