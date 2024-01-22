from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
from PyQt5.QtGui import *
import sys
import os
import numpy as np
import yaml
import pyqtgraph as pg
from trajectory_client import trajectory_node
import rclpy
from radio_process import Radio_Worker
from trajectory_msg.srv import Trajectory, Feedback

from path import Path


from PyQt5.QtWidgets import (

    QApplication,
    QFileDialog,
    QDialog,
    QPushButton,

    QVBoxLayout,
    QWidget,
    QGridLayout

)

from installer_ui import Installer_window
from PyQt5.QtCore import QObject, QThread, pyqtSignal
import subprocess
class Window(QWidget):

    def __init__(self):

        super().__init__()
        rclpy.init() ## I love ROS2


        self.installer = None
        self.setWindowTitle("Ultimate fleet manager")

        self.resize(800, 500)

        self.vehicle_configs = {}

        main_layout = QGridLayout(self)

        self.setLayout(main_layout)



        ##Variable declaration:


        self.selected_vehicle = None

        self.config_path = os.path.join(os.getcwd(), "configs")

        self.params = None

        self.selected_trajectory = None

        self.server_ip = "192.168.2.141"


        ##Widget declaration:

        self.CONFIG_PATH_LABEL = QLabel()

        self.CHANGE_CONFIG_PATH_BUTTON = QPushButton("Change")

        self.PLOT_GRAPH = pg.PlotWidget()

        self.EXUCUTE_BUTTON = QPushButton("Execute")
        self.EXUCUTE_BUTTON.setMaximumWidth(150)
        
        self.ADD_VEHICLE_BUTTON = QPushButton("New vehicle")

        self.REMOVE_VEHICLE_BUTTON = QPushButton("Remove vehicle")

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

        self.setMinimumWidth(300)
        ##Addind widgets to layout
        main_layout.addWidget(self.CONFIG_PATH_LABEL, 0,0)
        main_layout.addWidget(self.CHANGE_CONFIG_PATH_BUTTON, 0,1, alignment= QtCore.Qt.AlignmentFlag.AlignLeft)
        main_layout.addWidget(self.VEHICLE_LIST, 1,0, 2,1)
        main_layout.addWidget(self.PARAM_EDIT_BUTTON, 1,1)
        main_layout.addWidget(self.RELOAD_PARAM_BUTTON, 2,1, 2,1, alignment= QtCore.Qt.AlignmentFlag.AlignTop)
        main_layout.addWidget(self.ADD_VEHICLE_BUTTON, 3,0, alignment= QtCore.Qt.AlignmentFlag.AlignLeading)
        main_layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3,0, alignment= QtCore.Qt.AlignmentFlag.AlignTrailing)
        main_layout.addWidget(self.INSTALL_BUTTON, 2,1, 2,1, alignment= Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.TRAJECTORY_LIST, 0,2, 1,3)
        main_layout.addWidget(self.PLOT_GRAPH, 1,2, 2, 3, Qt.AlignmentFlag.AlignCenter)
        main_layout.addWidget(self.TRAJECTORY_LABEL, 4,2)
        main_layout.addWidget(self.EXUCUTE_BUTTON, 4,3)
        main_layout.addWidget(self.PROGRESSBAR, 5,2, 5,3)
        ##Connecting Widgets to functions:

        self.CHANGE_CONFIG_PATH_BUTTON.clicked.connect(self.config_chooser)

        self.TRAJECTORY_LIST.itemDoubleClicked.connect(self.plot_trajectory)

        self.VEHICLE_LIST.itemDoubleClicked.connect(self.vehicle_list_clicked_event)

        self.PARAM_EDIT_BUTTON.clicked.connect(self.param_button_clicked_event)

        self.RELOAD_PARAM_BUTTON.clicked.connect(self.set_config_file)

        self.EXUCUTE_BUTTON.clicked.connect(self.execute_trajectory)
        self.INSTALL_BUTTON.clicked.connect(self.install_onboard)
        ##Loading files:
        self.set_config_file() # This must be here because the function modifies the label text :(

    


    def install_onboard(self):
        if self.selected_vehicle == None:
            return
        self.installer = Installer_window(self.selected_vehicle)


    def edit_progressbar(self, request, response):
        self.PROGRESSBAR.setValue(request.progress)



        response.received = True
        return response
        


    def execute_trajectory(self):
        if self.selected_vehicle == None or self.selected_trajectory == None:
            print("Select vehicle & trajectory")
            return

        data = np.load(os.path.join(self.config_path, self.TRAJECTORY_LIST.selectedItems()[0].text()))

        p = Path(data)

        self.vehicle_configs[self.selected_vehicle]["trajectory_client"].send_request(p)
        self.vehicle_configs[self.selected_vehicle]["trajectory_client"].progress_srv =   self.vehicle_configs[self.selected_vehicle]["trajectory_client"].create_service(Feedback, self.selected_vehicle+"_vehicle_feedback",self.edit_progressbar )



    def param_button_clicked_event(self):
        if self.selected_vehicle != None:
            
            subprocess.Popen(['xdg-open', os.path.join(self.config_path,self.selected_vehicle + ".yaml")])
            subprocess.Popen(['xdg-open', os.path.join(self.config_path,self.selected_vehicle + "_login.yaml")])
        else:
            return
        



    def vehicle_list_clicked_event(self):

        v = self.VEHICLE_LIST.item(self.VEHICLE_LIST.currentRow(),0).text()
        
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
        if self.selected_trajectory == None:
            self.TRAJECTORY_LABEL.setText("<trajectory> -> " + self.selected_vehicle)
        else:
            self.TRAJECTORY_LABEL.setText( self.selected_trajectory+ " -> " + self.selected_vehicle)

    def plot_trajectory(self):
        self.selected_trajectory = self.TRAJECTORY_LIST.selectedItems()[0].text()


        if self.selected_vehicle == None:

            self.TRAJECTORY_LABEL.setText( self.selected_trajectory+" -> <vehicle>")
        else:
            self.TRAJECTORY_LABEL.setText(   self.selected_trajectory+ " -> "+ self.selected_vehicle)


        data = np.load(os.path.join(self.config_path, self.TRAJECTORY_LIST.selectedItems()[0].text()))

        self.PLOT_GRAPH.plot(data[:,0], data[:,1])

    def config_chooser(self):
        dlg = QFileDialog()
        dlg.setFileMode(QFileDialog.DirectoryOnly)


        if dlg.exec_():
            self.config_path = dlg.selectedFiles()[0]
        self.set_config_file()

    def set_config_file(self):
        self.CONFIG_PATH_LABEL.setText("config path: " + self.config_path)
        try:
            with open(os.path.join(self.config_path, "param.yaml"), 'r') as config:
                self.params = yaml.load(config, Loader= yaml.FullLoader)
            #Getting vehicle params
            vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
            for v in vehicles:
                self.vehicle_configs[v] = yaml.load(open( self.config_path + "/"+v+ ".yaml", "r"), Loader= yaml.FullLoader)
                self.vehicle_configs[v]["active"] = False
                self.vehicle_configs[v]["radio_thread"] = None
            self.load_vehicle_list()
            #Getting trajectories in the
            l = os.listdir(self.config_path)
            l = list(filter(lambda f: ".npy" in f, l))
            self.TRAJECTORY_LIST.clear()
            for t in l:
                self.TRAJECTORY_LIST.addItem(QListWidgetItem(t))
                
        except:
            raise FileNotFoundError("The folder must contain a param.yaml file and <vehicle_name>.yaml")
        
        

    def load_vehicle_list(self):
        self.VEHICLE_LIST.setColumnCount(3)
        self.VEHICLE_LIST.setRowCount(1)
        self.VEHICLE_LIST.setItem(0,0,QTableWidgetItem("ID"))
        vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        row = 0
        self.VEHICLE_LIST.setRowCount(len(vehicles))
        i = 0
        for v in vehicles:
            
            obj = Radio_Worker(obj_name=v,
                               ip=self.server_ip,devid= i, 
                               channel= int(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"]))
            
            
           
            
            self.vehicle_configs[v]["radio"] = obj



            self.vehicle_configs[v]["trajectory_client"] = trajectory_node(vehicle_name=v)



            
            self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
            self.VEHICLE_LIST.setItem(row,1, QTableWidgetItem(str(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"])))   
            self.VEHICLE_LIST.setItem(row,2, QTableWidgetItem("OFF"))
            for j in range(self.VEHICLE_LIST.columnCount()):
                self.VEHICLE_LIST.item(row, j).setBackground(Qt.GlobalColor.red)
                self.VEHICLE_LIST.item(row, j).setFlags(QtCore.Qt.ItemIsEnabled)
            row+= 1

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = Window()

    window.show()

    sys.exit(app.exec_())