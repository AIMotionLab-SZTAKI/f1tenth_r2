from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer
from PyQt5 import QtCore
from PyQt5.QtGui import *
import sys
import os
import numpy as np
import yaml
import pyqtgraph as pg
import rclpy
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QPushButton,
    QWidget,
    QGridLayout,
)
import subprocess
import shutil
import pickle
from scipy.interpolate import splev

from aimotion_fleet_manager.remote_window import controller
from aimotion_fleet_manager.installer_ui import Installer_Thread

from aimotion_fleet_manager.utils.radio_process import Radio_Worker
from trajectory_msg.srv import Trajectory, Feedback
from aimotion_fleet_manager.utils.Remote_Controller import ControlPublisher
from aimotion_fleet_manager.utils.path import Path
from aimotion_fleet_manager.utils.ros_classes import ROS_2_process
from aimotion_fleet_manager.utils.TCP_process import TCP_Server_process
from aimotion_fleet_manager.utils.path import Path
from aimotion_fleet_manager.utils.ip_tool import get_ip_address

import logging

import time
import atexit

class Window(QWidget):

    def __init__(self):
        super(Window, self).__init__()
        rclpy.init() ## I love ROS2

        self.setWindowIcon(QIcon(os.path.dirname(__file__)+"/images/icon.jpeg"))
        self.setWindowTitle("F1tenth fleet manager")
        self.resize(800, 500)

        
        self.main_layout = QGridLayout(self)
        self.setLayout(self.main_layout)


        ##Creating logger

        self.logger = logging.getLogger(__name__)

        # Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        self.logger.setLevel(logging.DEBUG)

        # Create a formatter
        formatter = logging.Formatter('[%(module)s] - %(levelname)s : %(message)s')

        # Create a console handler and set the formatter
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)

        # Add the console handler to the logger
        self.logger.addHandler(console_handler)





        
        ## Creating TCP Server:

        self.logger.info(get_ip_address())
        self.TCP = TCP_Server_process(host= get_ip_address(),
                                      port = 8000,
                                      message_callback= self.tcp_callback)
        self.TCP.start()
        self.logger.info("TCP server started")

        atexit.register(self.TCP.tcp_server.stop) ##Making sure that the connection is closed before ending process
        #atexit.register(self.TCP.tcp_server.server_socket.close) ##TODO not working!!!

        ##Variable declaration

        self.manual_enabled = False
        self.installer = None

        self.vehicle_configs = {}

        self.controller_window = controller()

        self.selected_vehicle = None

        self.config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "configs")

        ##Checking log folder existance:

        if os.path.exists(os.path.join(os.path.dirname(os.path.dirname(__file__)),"logs/")) == False:

            os.mkdir(os.path.join(os.path.dirname(os.path.dirname(__file__)),"logs/"))
        else:
            self.empty_log() ### Deleting previous logs

        if os.path.exists(os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories")) == False:
            os.mkdir(os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories"))
        

        with open(os.path.join(self.config_path, "param.yaml"), 'r') as config:
                self.params = yaml.load(config, Loader= yaml.FullLoader)
        

        self.selected_trajectory = None

        self.server_ip = self.params["server_ip"] #Set by the value in param.yaml

        #Creating timer for the remote controller->timout: sends msg to the car throught the ros2 node

        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.emit_controler)

        ##Widget declaration:
        self.CONFIG_PATH_LABEL = QLabel("config path: " + self.config_path)

        self.CHANGE_CONFIG_PATH_BUTTON = QPushButton("Change")
        
        self.IP_LABEL = QLabel("Server: ")

        self.IP_ADDRESS_TEXTBOX = QLineEdit(self.params["server_ip"])

        self.PLOT_GRAPH = pg.PlotWidget()

        self.EXUCUTE_BUTTON = QPushButton("Execute")
        self.EXUCUTE_BUTTON.setMaximumWidth(150)
        
        self.ADD_VEHICLE_BUTTON = QPushButton("Add")
        self.ADD_VEHICLE_BUTTON.setMaximumWidth(80)
        self.ADD_VEHICLE_BUTTON.setMinimumWidth(80)


        self.REMOVE_VEHICLE_BUTTON = QPushButton("Remove")
        self.REMOVE_VEHICLE_BUTTON.setMaximumWidth(80)
        self.REMOVE_VEHICLE_BUTTON.setMinimumWidth(80)


        self.PROGRESSBAR = QProgressBar()
        self.PROGRESSBAR.setMaximum(100)

        self.TRAJECTORY_LABEL = QLabel("<trajectory> -> <vehicle>")
        self.TRAJECTORY_LABEL.setMinimumWidth(320)
        self.TRAJECTORY_LABEL.setMaximumWidth(320)


        self.INSTALL_BUTTON = QPushButton("Install")

        self.PARAM_EDIT_BUTTON = QPushButton("Parameters")
        self.RELOAD_PARAM_BUTTON = QPushButton("Load")


        self.VEHICLE_LIST =QTableWidget()
        self.VEHICLE_LIST.verticalHeader().setVisible(False)
        self.VEHICLE_LIST.horizontalHeader().setVisible(False)
        self.VEHICLE_LIST.setMaximumWidth(300)
        
        self.TRAJECTORY_LIST = QListWidget()

        self.OK = QPushButton("OK")
        self.CANCEL = QPushButton("CANCEL")

        self.TEXTBOX = QLineEdit()
        self.TEXTBOX.setMaximumWidth(100)
        self.TEXTBOX.setMinimumWidth(100)

        self.MANUAL_BUTTON = QPushButton("MANUAL")
        self.MANUAL_BUTTON.setStyleSheet("background-color: red")
        self.MANUAL_BUTTON.setMaximumWidth(80)
        self.MANUAL_BUTTON.setMinimumWidth(80)


        self.TOGGLE_SAVE = QPushButton("Toggle save")


        self.setMinimumWidth(800)
        self.setMaximumWidth(800)
        self.setMinimumHeight(600)
        self.setMaximumHeight(600)

        ##Addind widgets to layout  
        self.main_layout.addWidget(self.CONFIG_PATH_LABEL, 0,0, alignment= QtCore.Qt.AlignmentFlag.AlignTop)
        self.main_layout.addWidget(self.CHANGE_CONFIG_PATH_BUTTON, 0,1, alignment= QtCore.Qt.AlignmentFlag.AlignTop)

        self.main_layout.addWidget(self.IP_ADDRESS_TEXTBOX, 0,0, alignment= QtCore.Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignRight)
        self.main_layout.addWidget(self.IP_LABEL, 0,0, alignment= QtCore.Qt.AlignmentFlag.AlignBottom| Qt.AlignmentFlag.AlignLeft)


        self.main_layout.addWidget(self.VEHICLE_LIST, 1,0, 2,1)
        self.main_layout.addWidget(self.PARAM_EDIT_BUTTON, 1,1)
        self.main_layout.addWidget(self.RELOAD_PARAM_BUTTON, 2,1, 2,1, alignment= QtCore.Qt.AlignmentFlag.AlignTop)
        self.main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        self.main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)
        self.main_layout.addWidget(self.INSTALL_BUTTON, 2,1, 2,1, alignment= Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.TRAJECTORY_LIST, 0,2, 1,3)
        self.main_layout.addWidget(self.PLOT_GRAPH, 1,2, 2, 3, Qt.AlignmentFlag.AlignCenter)
        self.main_layout.addWidget(self.TRAJECTORY_LABEL, 4,2)
        self.main_layout.addWidget(self.EXUCUTE_BUTTON, 4,3)
        self.main_layout.addWidget(self.PROGRESSBAR, 5,2, 5,3)
        self.main_layout.addWidget(self.MANUAL_BUTTON, 5,0, alignment= Qt.AlignmentFlag.AlignLeft)
        #self.main_layout.addWidget(self.TOGGLE_SAVE, 3,1 )

        ##Connecting Widgets to functions:

        self.TOGGLE_SAVE.clicked.connect(self.toggle_save)

        self.CHANGE_CONFIG_PATH_BUTTON.clicked.connect(self.config_chooser)

        self.TRAJECTORY_LIST.itemDoubleClicked.connect(self.plot_trajectory)

        self.VEHICLE_LIST.itemDoubleClicked.connect(self.vehicle_list_clicked_event)

        self.PARAM_EDIT_BUTTON.clicked.connect(self.param_button_clicked_event)

        self.RELOAD_PARAM_BUTTON.clicked.connect(self.Load_config_file)

        self.EXUCUTE_BUTTON.clicked.connect(self.execute_trajectory)
        self.INSTALL_BUTTON.clicked.connect(self.install_onboard)
        self.ADD_VEHICLE_BUTTON.clicked.connect(self.add_new_vehicle)
        
        self.REMOVE_VEHICLE_BUTTON.clicked.connect(self.remove_vehicle)

        self.MANUAL_BUTTON.clicked.connect(self.manual_mode_change)

        self.IP_ADDRESS_TEXTBOX.textChanged.connect(self.ip_check)
        
        
        ##Loading files:
        #self.set_config_file() # This must be here because the function modifies the label text :(
        

    def toggle_save(self):
        if self.selected_vehicle == None:
            return
        self.vehicle_configs[self.selected_vehicle]["ROS2"].toggle_save()

    def tcp_callback(self, message):
        #self.logger.info( message)
        match message["command"]:
        # vehicle verification
            
            case "get_logs":

                car_ID = message["car_ID"]

                l = os.listdir(os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs/"))
                l = list(filter(lambda f: car_ID in f, l)) #filtering files by vehicle_ID
                l_1 = []

                response = {"status": True, "files": []}
                for i in l:
                    if i.__contains__("_temp"): continue
                    l_1.append(i)
                    n_i = os.path.abspath(os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs/", i))
                    with open(n_i, "rb") as f:
                        response[i] = f.read()


                

                
                response["files"] = l_1

                return response
            
            case "list_trajectories":
                response= {"status": True
                    ,"trajectories": list()}
                
                response["trajectories"] = list(self.TRAJECTORY_LIST.item(i).text() for i in range(self.TRAJECTORY_LIST.count()))
                
                return response


            
            case "reload":
                self.Load_config_file()
                return{"status": True}
            


            case "verify_vehicle":
                if message["car_ID"] in self.vehicle_configs.keys():
                    message["status"] = True
                else:
                    message["status"] = False
                return message
            case "new_log":

                vehicle_id = message["car_ID"]

                self.vehicle_configs[vehicle_id]["ROS2"].logging_status = False

                self.vehicle_configs[vehicle_id]["ROS2"].toggle_save()



            case "logging":


                vehicle_id = message["car_ID"]
                switch = message["ON"]
                try:
                    self.vehicle_configs[vehicle_id]["ROS2"].node.logging_status = switch
                    return{"status": True}
                except Exception as e:
                    return{"status": False, "error": e}




            case "activate":
                """
                Little complicated stuff
                Basically I copied the first part of the vehicle_list_clicked_event
                This way the UI will show the change of the vehicle's radio status
                >:)
                """

                vehicle_id = message["car_ID"]


                if not vehicle_id in self.vehicle_configs.keys():
                    return {"status": False,
                            "error": "Vehicle not found"}
                switch = message["ON"]
                if switch == True:

                
                    try:
                        #If the vehicle is already running than we can skip the whole process
                        if self.vehicle_configs[vehicle_id]["active"] == True:
                            return{"status" : True}

                        self.vehicle_configs[vehicle_id]["active"] = True

                        ##Trying to start
                        self.vehicle_configs[vehicle_id]["radio"].start()


                        current_row = None

                        for i in range(self.VEHICLE_LIST.rowCount()):
                            if self.VEHICLE_LIST.item(i, 0).text() == vehicle_id:
                                current_row = i
                                break


                        ##Changing color of the current row:
                            

                        for i in range(self.VEHICLE_LIST.columnCount()):
                            self.VEHICLE_LIST.item(current_row, i).setBackground(Qt.GlobalColor.green)
                

                        self.VEHICLE_LIST.item(current_row, 2).setText("ON")
                        return{"status": True}
                    except Exception as e:
                        self.logger.error(e)
                        return {"status": False, "error": e}
                else:
                    if self.vehicle_configs[vehicle_id]["active"] == False:
                        return{"status" : True}
                    

                    try:
                        
                        self.vehicle_configs[vehicle_id]["active"] = False
               
                        self.vehicle_configs[vehicle_id]["radio"].stop()


                        current_row = None

                        for i in range(self.VEHICLE_LIST.rowCount()):
                            if self.VEHICLE_LIST.item(i, 0).text() == vehicle_id:
                                current_row = i
                                break


                        ##Changing color of the current row:
                            

                        for i in range(self.VEHICLE_LIST.columnCount()):
                            self.VEHICLE_LIST.item(current_row, i).setBackground(Qt.GlobalColor.red)
                

                        self.VEHICLE_LIST.item(current_row, 2).setText("OFF")
                        return{"status": True}
                    except Exception as e:
                        return {"status": False, "error": e}
            
            case "get_state":
                response={}
                vehicle_id = message["car_ID"]

                #Checking if vehicle is in the fleet:
                if not vehicle_id in self.vehicle_configs.keys():
                    response= {"status": False,
                               "error": "Vehicle not found"}
                    return response
                
                #Checking if the vehicles logging is active
                if self.vehicle_configs[vehicle_id]["active"] == False:
                    response = {"status": False,
                                "error": "Vehicle not active"}
                    return response
                
                #I don't use else if because of the return calling
                
                response["status"] = True
                response["state"] = self.vehicle_configs[vehicle_id]["ROS2"].node.state
                return response
        
        # send trajectory
            case "upload_trajectory":
                #recieved_trajectories.append(message["trajectory_ID"])
                path = {"pos_tck": message["pos_tck"],
                        "evol_tck": message["evol_tck"]}
                traj_id = message["trajectory_ID"]
                with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories/", traj_id + ".traj"), "wb") as file:
                    pickle.dump(obj=path, file= file)



                self.Load_config_file()
                return {"status": True}
        

            case "execute_trajectory":
                #time.sleep(5)
                
                response = {"status": False, "error": "Trajectory not found!"}
                if not message["car_ID"] in self.vehicle_configs.keys():
                    response["error"] = "vehicle not found"
                    return
                


                for i in range(self.TRAJECTORY_LIST.count()):
                    if self.TRAJECTORY_LIST.item(i).text() == (message["trajectory_ID"]+".traj"):
                        
                        #saving current selection 
                        t_v = self.selected_vehicle
                        t_t = self.selected_trajectory


                        self.selected_vehicle = message["car_ID"]
                        self.selected_trajectory = message["trajectory_ID"]+".traj"

                        #Calling the main_ui's execute trajectory method with the proper settings
                        self.execute_trajectory()


                        #Restoring original settings made by the user:
                        self.selected_vehicle = t_v
                        self.selected_trajectory = t_t


                        response = {"status": True, "error": "None"}
                        break
                return response
            
            case "upload_action":
                pass
                #recieved_actions.append(message["action_ID"])
                return {"status": True}
        
            case "execute_action": 
                """
                if message["action_ID"] in recieved_actions:
                    time.sleep(10) # wait to emulate execution
                    return {"status": True}
                else:
                    return {"status": False, "error": "Action not found!"}
                """
                pass
            case _:
                return {"status": False, "error": "Invalid command!"}
        return message



        

    def ip_check(self):
        """
        checks if the IP_ADDRESS_TEXTBOX value is set properly
        """
        address_text = self.IP_ADDRESS_TEXTBOX.text()
        address_text = address_text.replace('.', "")
        if address_text.isnumeric():
            self.IP_ADDRESS_TEXTBOX.setStyleSheet("background-color: white")
            self.server_ip = self.IP_ADDRESS_TEXTBOX.text()
            self.RELOAD_PARAM_BUTTON.setEnabled(True)
        else:
            self.IP_ADDRESS_TEXTBOX.setStyleSheet("background-color: red")
            self.RELOAD_PARAM_BUTTON.setDisabled(True)

    def emit_controler(self):


        self.vehicle_configs[self.selected_vehicle]["remote_controller"].publish(
            delta = float(self.controller_window.rot),
              duty_cycle = float(self.controller_window.forward))

    def manual_mode_change(self)->None:
        """
        Switch for manual mode
        """


        if self.manual_enabled == True or self.selected_vehicle == None:
            self.manual_enabled = False
            self.MANUAL_BUTTON.setStyleSheet("background-color: red")
            
            self.EXUCUTE_BUTTON.setEnabled(True)
               

            self.controller_window.close()
            self.timer.stop()
            
        else:
            
            self.EXUCUTE_BUTTON.setDisabled(True)
            self.controller_window.show()
            self.manual_enabled = True
            self.MANUAL_BUTTON.setStyleSheet("background-color: green")
            self.timer.start(int(1000/50))


    def remove_vehicle(self)->None:
        """
        Adds the additioal widgets to the UI with the right methods connected
        Arguments: None
        """


        self.main_layout.addWidget(self.TEXTBOX, 4,0, alignment= Qt.AlignmentFlag.AlignLeft)
        self.main_layout.addWidget(self.OK,4,0, alignment= Qt.AlignmentFlag.AlignCenter )
        self.main_layout.addWidget(self.CANCEL,4,0, alignment= Qt.AlignmentFlag.AlignRight )

        self.OK.setDisabled(True) 

        #Hidding add_vehicle and remove_vehicle buttons till canceling / adding the new vehicle
        self.ADD_VEHICLE_BUTTON.setParent(None)
        self.REMOVE_VEHICLE_BUTTON.setParent(None)
        
        self.TEXTBOX.textChanged.connect(self.vehicle_remove_block)

        self.OK.clicked.connect(self.ok_remove_vehicle)
        self.CANCEL.clicked.connect(self.cancel_remove_vehicle)

    def vehicle_remove_block(self)->None:
        """
        Checks wether the vehicle is in the fleet,
        if not than blocks OK button
        """


        vehicle_name= self.TEXTBOX.text()


        if vehicle_name in self.vehicle_configs.keys():
            self.OK.setEnabled(True)
        else:
            self.OK.setDisabled(True)


    def ok_remove_vehicle(self)->None:
        """
        Removes vehicle from the fleet's param.yaml and restores the UI to base view
        """
        




        vehicle_name = self.TEXTBOX.text()

        #Destroying the ROS2 nodes (idk why but killing their executors isn't enough, so we go try hard)
        self.vehicle_configs[vehicle_name]["remote_controller"].destroy_node()
        self.vehicle_configs[vehicle_name]["ROS2"].node.destroy_node()

        currentvehicles = list(self.params["vehicle_id_list"])
        currentvehicles.remove(vehicle_name)
        #Saving the modified param.yaml
        self.params["vehicle_id_list"] = currentvehicles
        with open(os.path.join("configs", "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()

        #Trying to removing the vehicles's config files:
        try:
            os.remove(os.path.join("configs", vehicle_name + ".yaml"))
            os.remove(os.path.join("configs", vehicle_name + "_login.yaml"))
        except Exception as error:
            self.logger.warn("Error removing the yaml files: \n" +  error)


        #reseting UI to base view:

        #Removing OK, TEXTBOX, CANCEL button from the UI


        self.OK.disconnect()
        self.CANCEL.disconnect()
        self.TEXTBOX.disconnect()


        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None) 
        self.TEXTBOX.setText("") #just makint sure that next time when adding new vehicle textbox is empty


        #Readding new vehicle and remove vehicle buttons to the UI:


        self.main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        self.main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)




    
    def cancel_remove_vehicle(self)->None:
        """
        Cancels the remove process and restores the UI to base view
        """

        #Removing OK, TEXTBOX, CANCEL button from the UI


        self.OK.disconnect()
        self.CANCEL.disconnect()
        self.TEXTBOX.disconnect()


        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None) 
        self.TEXTBOX.setText("") #just makint sure that next time when adding new vehicle textbox is empty


        #Readding new vehicle and remove vehicle buttons to the UI:


        self.main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        self.main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)



    def add_new_vehicle(self)-> None:
        """
        Adds the additioal widgets to the UI with the right methods connected
        Arguments: None
        """
        self.main_layout.addWidget(self.TEXTBOX, 4,0, alignment= Qt.AlignmentFlag.AlignLeft)
        self.main_layout.addWidget(self.OK,4,0, alignment= Qt.AlignmentFlag.AlignCenter )
        self.main_layout.addWidget(self.CANCEL,4,0, alignment= Qt.AlignmentFlag.AlignRight )

        self.OK.setDisabled(True)
        #Hidding add_vehicle and remove_vehicle buttons till canceling / adding the new vehicle
        self.ADD_VEHICLE_BUTTON.setParent(None)
        self.REMOVE_VEHICLE_BUTTON.setParent(None)
        
        self.TEXTBOX.textChanged.connect(self.vehicle_add_block)

        self.OK.clicked.connect(self.ok_new_vehicle)
        self.CANCEL.clicked.connect(self.cancel_new_vehicle)
    def ok_new_vehicle(self)->None:
        """
        This method creates the new yaml files for the new vehicle and adds it to the existing param.yaml list
        """


        vehicle_name = self.TEXTBOX.text()
        
        current_vehicles = list(self.params["vehicle_id_list"])

        current_vehicles.append(vehicle_name) ##Adding new vehicle to the fleet list

        ##Creating the parameter server yaml & login yaml for the new vehicle in the configs folder:
        shutil.copy(os.path.join(self.config_path, "Template.yaml"), os.path.join(self.config_path , vehicle_name +".yaml"))
        shutil.copy(os.path.join(self.config_path,  "Template_login.yaml"), os.path.join( self.config_path,vehicle_name +"_login.yaml"))


        #Removing OK, TEXTBOX, CANCEL button from the UI

        self.OK.disconnect()
        self.CANCEL.disconnect()
        self.TEXTBOX.disconnect()


        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None) 
        self.TEXTBOX.setText("") #just makint sure that next time when adding new vehicle textbox is empty

        
        #Readding new vehicle and remove vehicle buttons to the UI:


        self.main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        self.main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)

        #Saving the new param.yaml for the UI
        self.params["vehicle_id_list"] =  current_vehicles
        with open(os.path.join(self.config_path, "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()

        #Saving the new parameter server with the adjusted vehicle id
        with open(self.config_path + "/" +vehicle_name +".yaml", "r") as file:
            t_params = yaml.load(file, Loader=yaml.FullLoader)
            t_params["parameter_server"]["ros__parameters"]["car_id"] = str(vehicle_name)
            
            file.close()
            saver_file =  open(self.config_path + "/" +vehicle_name +".yaml", "w")
            yaml.dump(t_params, saver_file, default_flow_style=False)
            saver_file.close()

    def cancel_new_vehicle(self):
        """
        Cancel the adding process and restores the UI to the base view
        """

        #Removing OK, TEXTBOX, CANCEL button from the UI


        self.OK.disconnect()
        self.CANCEL.disconnect()
        self.TEXTBOX.disconnect()


        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None) 
        self.TEXTBOX.setText("") #just makint sure that next time when adding new vehicle textbox is empty


        #Readding new vehicle and remove vehicle buttons to the UI:


        self.main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        self.main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 4,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)
        
    def vehicle_add_block(self)->None:
        """
        Checks wether the vehicle is already in the fleet
        if the vehicle already exists then blocks adding it again
        """
        vehicle_name = self.TEXTBOX.text()
        
        #Checking if the vehicle name is proper for the rules
        if vehicle_name in self.vehicle_configs.keys() or vehicle_name == "" or vehicle_name.__contains__("."):
            self.OK.setDisabled(True)
        else:
            self.OK.setEnabled(True)
        
    def install_onboard(self)->None: 
        """
        Opens the installer window
        """
        if self.selected_vehicle == None:
            return
        self.installer = Installer_Thread(self.selected_vehicle)
        self.installer.start()

    def edit_progressbar(self, request, response):

        


        self.PROGRESSBAR.setValue(int(request.progress))
        if request.succeeded == True:

            #If the vehicle succeeded then end the log file
            vehicle_name = request.car_id
            self.vehicle_configs[self.selected_vehicle]["ROS2"].logging_status = False
            self.vehicle_configs[vehicle_name]["ROS2"].toggle_save()

            self.PROGRESSBAR.setValue(100)
        response.received = True
        return response
        


    def execute_trajectory(self):
        if self.selected_vehicle == None or self.selected_trajectory == None:
            self.logger.error("Select vehicle & trajectory")
            return

        #data = np.load(os.path.join(os.path.dirname(self.config_path), "trajectories", self.selected_trajectory))
        
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories", self.selected_trajectory), "rb") as file:
            data = pickle.load(file)
        
        #p = Path(data)
            
        # We start a new log file with toggle_save (which turns logging of, see in description)
           
        self.vehicle_configs[self.selected_vehicle]["ROS2"].toggle_save()


        #I turn logging back on before sending the trajectory on the controller

        self.vehicle_configs[self.selected_vehicle]["ROS2"].logging_status = True



        #The send request method is modified
        #TODO pls. do the testing after modifying the controller
        #Method definition in the utils/ros_classes.py

        self.vehicle_configs[self.selected_vehicle]["ROS2"].node.send_request(data)



    def param_button_clicked_event(self):
        if self.selected_vehicle != None:
            
            subprocess.Popen(['xdg-open', os.path.join(self.config_path,self.selected_vehicle + ".yaml")])
            subprocess.Popen(['xdg-open', os.path.join(self.config_path,self.selected_vehicle + "_login.yaml")])
        else:
            return
        



    def vehicle_list_clicked_event(self)->None:
        """
        Switches radio on/off and changes the selected vehicle parameter
        """


        v = self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(),0).text() #Temp
        
        if self.VEHICLE_LIST.currentColumn() == 2:

            if self.vehicle_configs[v]["active"] == False:

                self.vehicle_configs[v]["active"] = True
               
                ##Trying to start
                self.vehicle_configs[v]["radio"].start() 

                ##Changing color of the current row:
                for i in range(self.VEHICLE_LIST.columnCount()):
                    self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(), i).setBackground(Qt.GlobalColor.green)
                

                self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(), 2).setText("ON")
            
            else:

                self.vehicle_configs[v]["active"] = False
                self.vehicle_configs[v]["radio"].stop()
                
                for i in range(self.VEHICLE_LIST.columnCount()):

                    self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(), i).setBackground(Qt.GlobalColor.red)
                
                self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(), 2).setText("OFF")
                #TODO


            return
        


        #If not turning on/off streaming than vehicle select:


        self.selected_vehicle =  v # only change selected_vehicle if not trying to turn on / off
        self.controller_window.setWindowTitle(v+ " Remote controller")

        if self.selected_trajectory == None:
            self.TRAJECTORY_LABEL.setText("<trajectory> -> " + self.selected_vehicle)
        else:
            self.TRAJECTORY_LABEL.setText( self.selected_trajectory+ " -> " + self.selected_vehicle)

    def plot_trajectory(self):
        """
        Changes the displayed graph on the widget when new trajectory is selected from the listbox
        """

        self.PLOT_GRAPH.plotItem.clear()
        self.selected_trajectory = self.TRAJECTORY_LIST.selectedItems()[0].text()
        

        if self.selected_vehicle == None:

            self.TRAJECTORY_LABEL.setText(self.selected_trajectory+" -> <vehicle>")
        else:
            self.TRAJECTORY_LABEL.setText(self.selected_trajectory+ " -> "+ self.selected_vehicle)
        data = None


        """
        if self.selected_trajectory.__contains__(".npy"):
            data = np.load(os.path.join(os.path.dirname(os.path.dirname(__file__)),"trajectories", self.selected_trajectory))
            x = data[:,0]
            y = data[:,1]
        """ #Let's stick to only using pickle files (.traj)
        
        
        
        if self.selected_trajectory.__contains__(".traj"):
            file = open(os.path.join(os.path.dirname(os.path.dirname(__file__)),"trajectories", self.selected_trajectory), 'rb')
            data = pickle.load(file) ## data has ["pos_tck"] & ["evol_tck"] items stored

            pos_tck = data["pos_tck"]
            u = np.linspace(0, pos_tck[0][-1], 101)
            (x,y) = splev(u, pos_tck)

        self.PLOT_GRAPH.plot(x, y)
    def empty_log(self):
        """
        Cleans the log folder of the UI
        """
        log_path = os.path.join(os.path.dirname(os.path.dirname(__file__)),"logs/")
        for i in os.listdir(log_path):
            os.remove(os.path.join(log_path, i))

    def config_chooser(self):
        """
        Opens a file dialog in which the user can choose the new config directory
        """



        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.DirectoryOnly) 


        if dlg.exec_():
            self.config_path = dlg.selectedFiles()[0]
        self.Load_config_file()
    
    def Load_config_file(self):
        """
        setting new config file and loading vehicle list and vehicle parameters
        """
        self.RELOAD_PARAM_BUTTON.setText("Reload")

        self.CONFIG_PATH_LABEL.setText("config path: " + self.config_path)


        try:
            with open(os.path.join(self.config_path, "param.yaml"), 'r') as config:
                self.params = yaml.load(config, Loader= yaml.FullLoader)
            
            #Getting vehicle params
            vehicles = list(self.params["vehicle_id_list"])
            self.params["server_ip"] = self.server_ip 
            self.IP_ADDRESS_TEXTBOX.setText(self.server_ip)
            with open(os.path.join(self.config_path, "param.yaml"), "w") as file:
                yaml.dump(self.params, file, default_flow_style= False)
                file.close()

        
            
          

            for v in vehicles:
                #Turning off radio & destroying ROS2 nodes
                
                if self.vehicle_configs.__contains__(v):
                    try:
                        self.vehicle_configs[v]["radio"].stop()
                        self.vehicle_configs[v]["radio"].streamer.close()
                    except:
                        pass
                    try:
                        self.vehicle_configs[v]["ROS2"].stop()
                        self.vehicle_configs[v]["ROS2"].executor.shutdown()
                        self.vehicle_configs[v]["ROS2"].executor.remove_node(self.vehicle_configs[v]["ROS2"].node)
                        self.vehicle_configs[v]["remote_controller"].destroy_node()
                        self.vehicle_configs[v]["ROS2"].node.destroy_node()
                    except Exception as e:
                        self.logger.error(e)
                rclpy.shutdown()
                rclpy.init()
                


                self.vehicle_configs[v] = yaml.load(open( self.config_path + "/"+v+ ".yaml", "r"), Loader= yaml.FullLoader)
                self.vehicle_configs[v]["active"] = False
                self.vehicle_configs[v]["radio"] = None

            self.load_vehicle_list()

            #Reading trajectories from the config folder:
            l = os.listdir(os.path.join(os.path.dirname(os.path.dirname(__file__)), "trajectories"))
            #l = list(filter(lambda f: ".npy" in f or ".traj" in f, l)) #filtering file by .npy


            l = list(filter(lambda f: ".traj" in f, l)) #We only use pickle files (.traj)


            self.TRAJECTORY_LIST.clear()


            for t in l:
                self.TRAJECTORY_LIST.addItem(QListWidgetItem(t)) #Reloading the listbox
                
        except:
            raise FileNotFoundError("The folder must contain a param.yaml file and <vehicle_name>.yaml")
        
        

    def load_vehicle_list(self):
        """
        Loads the cells of the fleet table
        """

        
        self.PROGRESSBAR.setValue(0)


        #Getting all the vehicle names in the fleet:
        vehicles = list(self.params["vehicle_id_list"])



        
        self.VEHICLE_LIST.setRowCount(len(vehicles)) 
        self.VEHICLE_LIST.setColumnCount(3) # vehicle_id, channel, radio status



        row = 0
        i = 0
        for v in vehicles:

            #Creating a remote controller for every vehicle in the fleet:
            self.vehicle_configs[v]["remote_controller"] = ControlPublisher(vehicle_name= v)

            #Creating trajectory clients for every vehicle in the fleet, and starting them:
            self.vehicle_configs[v]["ROS2"] = ROS_2_process(vehicle_name=v)
            
            #Setting the callback for the trajectory node to show the progress on the progressbar
            self.vehicle_configs[v]["ROS2"].node.progress_srv =   self.vehicle_configs[v]["ROS2"].node.create_service(Feedback, v+"_vehicle_feedback",self.edit_progressbar)
            
            #Starting the thread:
            self.vehicle_configs[v]["ROS2"].start()


            try:
                obj = Radio_Worker(obj_name=v,
                               ip=self.server_ip,devid= i, 
                               channel= int(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"]))
            
            
           
            
                self.vehicle_configs[v]["radio"] = obj
            except Exception as e:
                self.logger.critical("Error with vehicle: "+ v )
                self.logger.warn(e)
                #self.logger.critical("There is not enough Crazy dongles inserted")
                
                for i in range(0,3):
                    self.VEHICLE_LIST.setItem(row,i, QTableWidgetItem(" "))
                    self.VEHICLE_LIST.item(row, i).setBackground(Qt.GlobalColor.yellow)

                self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
                self.VEHICLE_LIST.item(row, 0).setBackground(Qt.GlobalColor.yellow)
                self.VEHICLE_LIST.item(row, 2).setText("Unavailable")

                self.VEHICLE_LIST.item(row, 0).setFlags(QtCore.Qt.ItemIsEnabled)
                row+= 1
                continue

        
            


            #Some fancy stuff-> coloring the table and setting the texts of the cells
            self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
            self.VEHICLE_LIST.setItem(row,1, QTableWidgetItem(str(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"])))   
            self.VEHICLE_LIST.setItem(row,2, QTableWidgetItem("OFF"))
            for j in range(self.VEHICLE_LIST.columnCount()):
                self.VEHICLE_LIST.item(row, j).setBackground(Qt.GlobalColor.red)
                self.VEHICLE_LIST.item(row, j).setFlags(QtCore.Qt.ItemIsEnabled)
            row+= 1
            i+=1

def main():
    app = QApplication(sys.argv)

    window = Window()

    window.show()

    sys.exit(app.exec_())



if __name__ == "__main__":
    main()