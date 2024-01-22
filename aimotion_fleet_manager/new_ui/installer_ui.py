import paramiko
import os
import install_utils 
from install_utils import create_clients
import time
from pathlib import Path
import yaml
import PyQt5
from ament_index_python.packages import get_package_share_directory
#host = "192.168.2.62"
username = "f1tenth"
password = "123456" ## This will be commented out !!!
from PyQt5.QtWidgets import *
import shutil
import subprocess
import sys
import os
import threading




class Installer_window(QWidget):
    def __init__(self, vehicle_name):
        super().__init__()
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.output_rd = QTextBrowser()
        self.vehicle_name = vehicle_name
        layout.addWidget(self.output_rd)
        config = open("configs/" +vehicle_name + "_login.yaml", 'r')
        self.params = yaml.load(config, Loader= yaml.FullLoader)
        self.show()
        self.setMinimumWidth(300)
        self.setMinimumHeight(300)
        self.setWindowTitle(vehicle_name + " installation")
        #self.t = threading.Thread(target= self.start_install)
        #self.t.start()
        self.start_install()
    def start_install(self):
        self.onboard_path = os.path.join(Path(os.getcwd()).parents[1], "f1_car")
        #print(self.onboard_path)
        self.edit_parameter_server()
        host = self.params["IP"]
        username = self.params["Username"]
        password = self.params["Password"]
        try:    
            self.output_rd.append("Trying to connect")
            QApplication.processEvents()
            SSH_client, SFTP_client = create_clients(host, username, password)
        except Exception as error:
            self.output_rd.append("Failed to connect")
            QApplication.processEvents()
            return
        self.output_rd.append("Deleting existing workspace...")
        QApplication.processEvents()
        _stdin, stdout, stderr = SSH_client.exec_command("rm -rf aimotion-f1tenth-system")
        self.output_rd.append("Copying workspace onto vehicle...")
        QApplication.processEvents()
        #SFTP_client.rmall("aimotion-f1tenth-system")
        time.sleep(5)
        SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)
        wd = Path(os.getcwd())
#print(wd.parents[1])
        self.path = os.path.join(wd.parents[1], "f1_car")
        SFTP_client.put_dir(self.path, "aimotion-f1tenth-system")
       


        self.output_rd.append("Building ROS workspace...")
        QApplication.processEvents()
        _stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
        try:
            for line in iter(stdout.readline, ""):
                    self.output_rd.append(line)
                    self.output_rd.verticalScrollBar().setValue(self.output_rd.verticalScrollBar().maximum())
                    QApplication.processEvents()
            if stdout.channel.recv_exit_status():
                self.output_rd.append("Failed to build ROS workspace")
                QApplication.processEvents()
            else:
                self.output_rd.append("Successfully installed aimotion-f1tenth-system on vehicle")
                QApplication.processEvents()
        except:
            print("error while printing")
            QApplication.processEvents()
        SSH_client.close()
        SFTP_client.close()
    def edit_parameter_server(self):
        remove_path = os.path.join(self.onboard_path, "src", "param_server", "config", "param.yaml")
        #print(remove_path)
        try:
            os.remove(remove_path)
        except(Exception):
            pass
        #print(os.path.join(os.getcwd(), "configs",self.vehicle_name+".yaml" ))
        shutil.copyfile(os.path.join(os.getcwd(), "configs",self.vehicle_name+".yaml" ), remove_path)