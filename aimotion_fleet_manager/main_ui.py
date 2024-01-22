from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5 import QtCore
from PyQt5.QtGui import *
import sys
import os
import numpy as np
import yaml
import pyqtgraph as pg
from ros_classes import trajectory_node, trajectory_client_process
import rclpy
from radio_process import Radio_Worker
from trajectory_msg.srv import Trajectory, Feedback
from Remote_Controller import ControlPublisher
from path import Path


from PyQt5.QtWidgets import (

    QApplication,
    QFileDialog,
    QDialog,
    QPushButton,

    QVBoxLayout,
    QWidget,
    QGridLayout,
    QDoubleSpinBox

)

from installer_ui import Installer_window, Installer_Thread
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import subprocess
import shutil

from remote_window import controller

class Window(QWidget):

    def __init__(self):
        super(Window, self).__init__()
        #self.installEventFilter(self)
        # super().__init__()
        rclpy.init() ## I love ROS2
        self.setWindowIcon(QIcon("icon.jpeg"))
        self.setWindowTitle("F1tenth fleet manager")
        self.resize(800, 500)
        self.main_layout = QGridLayout(self)
        self.setLayout(self.main_layout)


        ##Variable declaration

        self.manual_enabled = False
        self.installer = None

        self.vehicle_configs = {}

        self.controller_window = controller()

        self.selected_vehicle = None

        self.config_path = os.path.join(os.path.dirname(os.getcwd()), "configs")

        self.params = None

        self.selected_trajectory = None

        self.server_ip = "192.168.2.141"

        self.timer = QTimer()
        self.timer.setTimerType(Qt.PreciseTimer)
        self.timer.timeout.connect(self.emit_controler)

        ##Widget declaration:
        self.CONFIG_PATH_LABEL = QLabel()

        self.CHANGE_CONFIG_PATH_BUTTON = QPushButton("Change")
        
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
        self.RELOAD_PARAM_BUTTON = QPushButton("Reload")


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


        self.setMinimumWidth(800)
        self.setMaximumWidth(800)
        self.setMinimumHeight(600)
        self.setMaximumHeight(600)

        ##Addind widgets to layout  
        self.main_layout.addWidget(self.CONFIG_PATH_LABEL, 0,0)
        self.main_layout.addWidget(self.CHANGE_CONFIG_PATH_BUTTON, 0,1, alignment= QtCore.Qt.AlignmentFlag.AlignLeft)
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
        

        ##Connecting Widgets to functions:

        self.CHANGE_CONFIG_PATH_BUTTON.clicked.connect(self.config_chooser)

        self.TRAJECTORY_LIST.itemDoubleClicked.connect(self.plot_trajectory)

        self.VEHICLE_LIST.itemDoubleClicked.connect(self.vehicle_list_clicked_event)

        self.PARAM_EDIT_BUTTON.clicked.connect(self.param_button_clicked_event)

        self.RELOAD_PARAM_BUTTON.clicked.connect(self.set_config_file)

        self.EXUCUTE_BUTTON.clicked.connect(self.execute_trajectory)
        self.INSTALL_BUTTON.clicked.connect(self.install_onboard)
        self.ADD_VEHICLE_BUTTON.clicked.connect(self.add_new_vehicle)
        
        self.REMOVE_VEHICLE_BUTTON.clicked.connect(self.remove_vehicle)

        self.MANUAL_BUTTON.clicked.connect(self.manual_mode_change)
 
        ##Loading files:
        self.set_config_file() # This must be here because the function modifies the label text :(
    

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


        currentvehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        currentvehicles.remove(vehicle_name)

        #Saving the modified param.yaml
        self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"] = currentvehicles
        with open(os.path.join("configs", "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()

        #Trying to removing the vehicles's config files:
        try:
            os.remove(os.path.join("configs", vehicle_name + ".yaml"))
            os.remove(os.path.join("configs", vehicle_name + "_login.yaml"))
        except Exception as error:
            print("Error removing the yaml files: \n", error)


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
        
        current_vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])

        current_vehicles.append(vehicle_name) ##Adding new vehicle to the fleet list

        ##Creating the parameter server yaml & login yaml for the new vehicle in the configs folder:
        shutil.copy(os.path.join(os.getcwd(), "configs", "Template.yaml"), os.path.join(os.getcwd(), "configs",  vehicle_name +".yaml"))
        shutil.copy(os.path.join(os.getcwd(), "configs", "Template_login.yaml"), os.path.join(os.getcwd(), "configs",  vehicle_name +"_login.yaml"))


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
        self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"] =  current_vehicles
        with open(os.path.join("configs", "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()

        #Saving the new parameter server with the adjusted vehicle id
        with open("configs/"+vehicle_name +".yaml", "r") as file:
            t_params = yaml.load(file, Loader=yaml.FullLoader)
            t_params["parameter_server"]["ros__parameters"]["car_id"] = str(vehicle_name)
            
            file.close()
            saver_file =  open("configs/"+vehicle_name +".yaml", "w")
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
            self.PROGRESSBAR.setValue(100)
        response.received = True
        return response
        


    def execute_trajectory(self):
        if self.selected_vehicle == None or self.selected_trajectory == None:
            print("Select vehicle & trajectory")
            return

        data = np.load(os.path.join(self.config_path, self.TRAJECTORY_LIST.selectedItems()[0].text()))

        p = Path(data)

        self.vehicle_configs[self.selected_vehicle]["trajectory_client"].node.send_request(p)



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


        self.selected_trajectory = self.TRAJECTORY_LIST.selectedItems()[0].text()


        if self.selected_vehicle == None:

            self.TRAJECTORY_LABEL.setText( self.selected_trajectory+" -> <vehicle>")
        else:
            self.TRAJECTORY_LABEL.setText(   self.selected_trajectory+ " -> "+ self.selected_vehicle)


        data = np.load(os.path.join(self.config_path, self.TRAJECTORY_LIST.selectedItems()[0].text()))

        self.PLOT_GRAPH.plot(data[:,0], data[:,1])

    def config_chooser(self):
        """
        Opens a file dialog in which the user can choose the new config directory
        """



        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.DirectoryOnly) 


        if dlg.exec_():
            self.config_path = dlg.selectedFiles()[0]
        self.set_config_file()

    def set_config_file(self):
        """
        setting new config file and loading vehicle list and vehicle parameters
        """

        self.CONFIG_PATH_LABEL.setText("config path: " + self.config_path)


        try:
            with open(os.path.join(self.config_path, "param.yaml"), 'r') as config:
                self.params = yaml.load(config, Loader= yaml.FullLoader)
            
            #Getting vehicle params
            vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
            for v in vehicles:
                #Turning off radio & destroying ROS2 nodes
                if self.vehicle_configs.__contains__(v):
                    try:
                        self.vehicle_configs[v]["radio"].stop()
                        self.vehicle_configs[v]["radio"].streamer.close()
                    except:
                        pass

                    try:
                        self.vehicle_configs[v]["remote_controller"].destroy_node()
                    except:
                        pass
                    try:
                        self.vehicle_configs[v]["trajectory_client"].node.destroy_node()
                        self.vehicle_configs[v]["trajectory_client"].trajectory_client.destroy()
                    except:
                        pass
                

                rclpy.shutdown()
                rclpy.init()

                self.vehicle_configs[v] = yaml.load(open( self.config_path + "/"+v+ ".yaml", "r"), Loader= yaml.FullLoader)
                self.vehicle_configs[v]["active"] = False
                self.vehicle_configs[v]["radio"] = None

            self.load_vehicle_list()

            #Reading trajectories from the config folder:
            l = os.listdir(self.config_path)
            l = list(filter(lambda f: ".npy" in f, l)) #filtering file by .npy



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
        vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])



        
        self.VEHICLE_LIST.setRowCount(len(vehicles)) 
        self.VEHICLE_LIST.setColumnCount(3) # vehicle_id, channel, radio status



        row = 0
        i = 0
        for v in vehicles:
            try:
                obj = Radio_Worker(obj_name=v,
                               ip=self.server_ip,devid= i, 
                               channel= int(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"]))
            
            
           
            
                self.vehicle_configs[v]["radio"] = obj
            except Exception as e:
                print("Error with vehicle: ", v )
                print(e)
                print("There is not enough Crazy dongles inserted")
                self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
                self.VEHICLE_LIST.item(row, 0).setFlags(QtCore.Qt.ItemIsEnabled)
                row+= 1
                continue

        

            self.vehicle_configs[v]["remote_controller"] = ControlPublisher(vehicle_name= v)


            self.vehicle_configs[v]["trajectory_client"] = trajectory_client_process(vehicle_name=v)
            self.vehicle_configs[v]["trajectory_client"].node.progress_srv =   self.vehicle_configs[v]["trajectory_client"].node.create_service(Feedback, v+"_vehicle_feedback",self.edit_progressbar)

            self.vehicle_configs[v]["trajectory_client"].start()

            
            self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
            self.VEHICLE_LIST.setItem(row,1, QTableWidgetItem(str(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"])))   
            self.VEHICLE_LIST.setItem(row,2, QTableWidgetItem("OFF"))
            for j in range(self.VEHICLE_LIST.columnCount()):
                self.VEHICLE_LIST.item(row, j).setBackground(Qt.GlobalColor.red)
                self.VEHICLE_LIST.item(row, j).setFlags(QtCore.Qt.ItemIsEnabled)
            row+= 1
            i+=1

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = Window()

    window.show()

    sys.exit(app.exec_())