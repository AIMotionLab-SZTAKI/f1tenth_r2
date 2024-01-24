import paramiko
import os
from .utils.install_utils import create_clients
import time
from pathlib import Path
import yaml
import PyQt5
from ament_index_python.packages import get_package_share_directory
from PyQt5.QtWidgets import *
import shutil

import os
from PyQt5.QtCore import QThread


class Installer_Thread(QThread):
    def __init__(self, vehicle_name):
        super(Installer_Thread, self).__init__()
        self.vehicle_name = vehicle_name
        


        
    def run(self):
        self.window = Installer_window(self.vehicle_name)
        self.window.show()
        self.window.onboard_path = os.path.join(Path(os.path.dirname(__file__)).parents[0], "ros2/f1_car")
        
        self.window.edit_parameter_server() 
        host = self.window.params["IP"]
        username = self.window.params["Username"]
        password = self.window.params["Password"]


        try:
             
            self.window.output_rd.append("Trying to connect")
            QApplication.processEvents()
            SSH_client, SFTP_client = create_clients(host, username, password)
        except Exception as error:
            self.window.output_rd.append("Failed to connect")
            QApplication.processEvents()
            return
        self.window.output_rd.append("Deleting existing workspace...")
        QApplication.processEvents()
        _stdin, stdout, stderr = SSH_client.exec_command("rm -rf aimotion-f1tenth-system") #removing previous package
        self.window.output_rd.append("Copying workspace onto vehicle...")
        QApplication.processEvents()



        time.sleep(5)
        SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)
        wd = Path(os.path.dirname(__file__))
        SFTP_client.put_dir(self.window.onboard_path, "aimotion-f1tenth-system")
       


        self.window.output_rd.append("Building ROS workspace...")
        QApplication.processEvents()
        _stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
        try:
            for line in iter(stdout.readline, ""):
                    self.window.output_rd.append(line)
                    self.window.output_rd.verticalScrollBar().setValue(self.window.output_rd.verticalScrollBar().maximum())
                    QApplication.processEvents()
            if stdout.channel.recv_exit_status():
                self.window.output_rd.append("Failed to build ROS workspace")
                QApplication.processEvents()
            else:
                self.window.output_rd.append("Successfully installed aimotion-f1tenth-system on vehicle")
                QApplication.processEvents()
        except:
            print("error while printing")
            QApplication.processEvents()
        SSH_client.close()
        SFTP_client.close()
        self.window.close()


class Installer_window(QWidget):
    """
    Installer window with a textbox to show the progress
    """

    def __init__(self, vehicle_name):
        super().__init__()
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.output_rd = QTextBrowser()
        self.vehicle_name = vehicle_name
        layout.addWidget(self.output_rd)
        config = open(os.path.join(os.path.dirname(os.path.dirname(__file__)),"configs/" +vehicle_name + "_login.yaml"), "r")
        self.params = yaml.load(config, Loader= yaml.FullLoader)
        self.onboard_path = None
        self.setMinimumWidth(300)
        self.setMinimumHeight(300)
        self.setWindowTitle(vehicle_name + " installation")


    def edit_parameter_server(self):
        remove_path = os.path.join(self.onboard_path, "src", "param_server", "config", "param.yaml")
        #print(remove_path)
        try:
            os.remove(remove_path)
        except(Exception):
            pass
        #print(os.path.join(os.path.dirname(__file__), "configs",self.vehicle_name+".yaml" ))
        shutil.copyfile(os.path.join(os.path.dirname(os.path.dirname(__file__)), "configs",self.vehicle_name+".yaml" ), remove_path)
        