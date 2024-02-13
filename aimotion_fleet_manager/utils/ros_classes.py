from trajectory_msg.srv import  EvolTrajectory
from vehicle_state_msgs.msg import VehicleStateStamped
from datetime import datetime #log file is saved by time
from rclpy.node import Node
from PyQt5.QtCore import  QThread
import rclpy
from std_srvs.srv import SetBool
import csv #used for logging
import os
import atexit
from scipy.interpolate import splev, splint

from rclpy.executors import SingleThreadedExecutor

from pathlib import Path

import logging

from aimotion_fleet_manager.utils.logging_formater import CustomFormatter

class ROS_2_process(QThread):
    """
    Provides start() func to run async the rclpy.spin for a trajectory_node instance
    """


    def __init__(self, vehicle_name):

        super(ROS_2_process, self).__init__()

        self.node = ROS2_node(vehicle_name=vehicle_name)
        self.executor = SingleThreadedExecutor()
        
        self.executor.add_node(self.node)

        self.vehicle_name = vehicle_name

        ##Creating logger

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)

        # create console handler with a higher log level
        ch = logging.StreamHandler()
        ch.setLevel(logging.DEBUG)

        ch.setFormatter(CustomFormatter())

        self.logger.addHandler(ch)


    def set_logging_status(self, status: bool):
        """
        Sets logging_status variable for the Node
        Args:
        - status: bool
        """


        self.node.logging_status = status
        self.logger.info("Logging status of vehicle: {0} is set to: {1}".format(self.vehicle_name, status))


    def toggle_save(self):
        self.node.toggle_save()
        
        self.logger.info("Log file of vehicle: {0} is saved".format(self.vehicle_name))

    def stop(self):
        self.executor.shutdown()
    def run(self):
        
        try:
            self.executor.spin()
        except: #In case of shutdown / reload this would give an error
            pass
        

class ROS2_node(Node):
    """
    Simple ROS2 node with
    - trajectory_client
    - unset progress_srv service
    - logger
    - logger node
    - etc.
    """
    def __init__(self, vehicle_name):

        super().__init__(vehicle_name+'_PC_client')

        self.trajectory_client = self.create_client(EvolTrajectory, vehicle_name+'_execute_trajectory') #CHANGE!!!! New service message type (old: Trajectory.srv<-> new: EvolTrajectory.srv) 
        
        self.controller_manager =  self.create_client(SetBool, vehicle_name+ "_controller_switch")



        #See new message type in the ros2/f1_PC/trajectory_msg/srv
        #I didn't delete the old msg type just in case

        
        self.progress_srv =  None#self.create_service(Feedback, vehicle_name+"_vehicle_feedback",self.feedback_callback )
        
        self.vehicle_name = vehicle_name
        


        #Creating the subscribtion on the vehicle's state node

        self.logging_status = False
      


        #making sure that self.destroy won't run multiple times when closing
        atexit.unregister(self.destroy)
        
        atexit.register(self.destroy)
        

        self.create_subscription(   
            msg_type=VehicleStateStamped,
            topic= "/"+vehicle_name+ "_state",
            callback= self.state_callback,
            qos_profile=1
            )

        



        # Creating the log writer etc.
        self.fieldnames = ['time_stamp_sec', 'position_x', 'position_y', 'heading_angle', 'velocity_x', 'velocity_y', 'omega', 'duty_cycle', 'delta', 'erpm']
        
        
        self.csv_file =  open(str(Path(__file__).parents[2])+"/logs/_temp_"+ vehicle_name +'.csv', mode='w') #opening the temporary file
        

        self.writer = csv.DictWriter(self.csv_file, fieldnames= self.fieldnames)
        
        self.writer.writeheader()
        self.state = {'time_stamp_sec': None,
        'position_x': None,
        'position_y': None,
        'velocity_x': None,
        'velocity_y': None,
        'heading_angle': None,
        'omega': None,
        'duty_cycle': None,
        'delta': None,
        'erpm': None,}

    def controller_switch(self, mode: bool):

        message = SetBool.Request()
        message.data = mode
        #print("Waiting for service....")
        #self.client.wait_for_service()
        self.future = self.controller_manager.call_async(message)

    def destroy(self):
        try:
            file_name = self.csv_file.name
            os.remove(file_name)
        except:
            pass #No file to delete

    def toggle_save(self):
        """
        Closes and saves the current csv_file and start a new
        """


        closed_file = self.csv_file.name
        self.csv_file.close()

        self.logging_status = False

        now = datetime.now()

        os.rename(closed_file, str(Path(__file__).parents[2])+"/logs/"+self.vehicle_name + now.strftime("_%m_%d_%Y_%H_%M_%S")+'.csv')
        
        self.csv_file = open(str(Path(__file__).parents[2])+"/logs/_temp_"+ self.vehicle_name +'.csv', mode='w') #opening the temporary file
        
        self.writer = csv.DictWriter(self.csv_file, fieldnames= self.fieldnames)
        self.writer.writeheader()


    def state_callback(self, data):
        self.state = {
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
                 }
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
            
    def send_request(self, path):

        request = EvolTrajectory.Request()
        
        

        request.path_t = []
        for i in range(len(path["pos_tck"][0])):
            request.path_t.append(float(path["pos_tck"][0][i]))
        request.path_cx = []
        for i in range(len(path["pos_tck"][1][0])):
            request.path_cx.append(float(path["pos_tck"][1][0][i]))
        request.path_cy = []
        for i in range(len(path["pos_tck"][1][1])):
            request.path_cy.append(float(path["pos_tck"][1][1][i]))
        request.path_k = path["pos_tck"][2]

        
        #Here comes the new version:


        request.evol_t = []

        for i in range(len(path["evol_tck"][0])):
            request.evol_t.append(float(path["evol_tck"][0][i]))
        
        request.evol_c = []

        for i in range(len(path["evol_tck"][1])):
            request.evol_c.append(float(path["evol_tck"][1][i]))
        
        request.evol_k = path["evol_tck"][2]

        request.reversed = path["reversed"] #This shows the vehicle if the trajectory is reversed


        # We still wanna start from 0 and go till the end of path so this won't change: This is calculated on the car :)

        
        #Sending request (same as in the old version):


        try:
            if self.trajectory_client.wait_for_service(timeout_sec= 2) == False:
                raise SystemError("Service is not running")
            self.future = self.trajectory_client.call_async(request)
        




        #rclpy.spin_until_future_complete(self, self.future) #I don't wanna wait till the response, it would freeze the GUI
            
        #print(self.future.result().received)
            

        except Exception as error :
            print(error)