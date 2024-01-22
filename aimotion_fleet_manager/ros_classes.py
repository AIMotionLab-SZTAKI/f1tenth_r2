from trajectory_msg.srv import Trajectory
from vehicle_state_msgs.msg import VehicleStateStamped
from datetime import datetime #log file is saved by time
from rclpy.node import Node
from PyQt5.QtCore import  QObject, QThread
import rclpy
import csv #used for logging
import os
import atexit

class logger_node_process(QThread):

    def __init__(self, vehicle_name):
        super(logger_node_process, self).__init__()
        self.node = logger_node(vehicle_name=vehicle_name)
        try:
            atexit.unregister(self.destroy)
        except:
            pass # no method to remove

        atexit.register(self.destroy)

    def run(self):
        rclpy.spin(self.node)
    def destroy(self):
        """
        destroys current temp file 
        """
        try:
            file_name = self.node.csv_file.name
            os.remove(file_name)
        except:
            pass #No file to delete


class logger_node(Node):
    def __init__(self, vehicle_name):
        super().__init__(vehicle_name + "_logger_node")
        self.vehicle_name = vehicle_name
        self.create_subscription(
            msg_type=VehicleStateStamped,
            topic= vehicle_name+ "_state",
            callback= self.state_callback,
            qos_profile=1
            )
        
        self.logging_status = False
        self.fieldnames = ['time_stamp_sec', 'position_x', 'position_y', 'heading_angle', 'velocity_x', 'velocity_y', 'omega', 'duty_cycle', 'delta', 'erpm']
        
        
        now = datetime.now() # current date and time

        self.csv_file =  open("logs/_temp_"+ vehicle_name +'.csv', mode='w') #opening the temporary file
        

        self.writer = csv.DictWriter(self.csv_file, fieldnames= self.fieldnames)
        
        self.writer.writeheader()


    def toggle_save(self):
        """
        Closes and saves the current csv_file and start a new
        """

        closed_file = self.csv_file.name
        self.csv_file.close()

        now = datetime.now()

        os.rename(closed_file, "logs/"+self.vehicle_name + now.strftime("_%m_%d_%Y_%H_%M_%S")+'.csv')
        
        self.csv_file = open("logs/_temp_"+self.vehicle_name+ ".csv", mode='w') #opening the temporary file
        
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
    """
    Simple ROS2 node with a trajectory_client and an unset progress_srv service
    """
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