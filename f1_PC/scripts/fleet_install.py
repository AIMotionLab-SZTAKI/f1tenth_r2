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
import sys
import os
#config = ("config/param.yaml")


class Window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        layout = QGridLayout()
        
        self.setLayout(layout)
        self.listwidget = QListWidget()
        q = 0
        config = os.path.join(get_package_share_directory('mocap_pkg'),  'param.yaml')
        config = open(config, 'r')
        self.params = yaml.load(config, Loader= yaml.FullLoader)
        for i in self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"]:
            self.listwidget.insertItem(q, i)
            q+=1
        """
        self.listwidget.insertItem(0, "Red")
        self.listwidget.insertItem(1, "Orange")
        self.listwidget.insertItem(2, "Blue")
        self.listwidget.insertItem(3, "White")
        self.listwidget.insertItem(4, "Green")
        """
        self.listwidget.itemDoubleClicked.connect(self.clicked)

        layout.addWidget(self.listwidget)
        wd = Path(os.getcwd())
#print(wd.parents[1])
        self.path = os.path.join(wd.parents[1], "f1_car")
    def clicked(self, qmodelindex):
        item = self.listwidget.currentItem()
        host = self.params["parameter_server"]["ros__parameters"][item.text()]["IP"]
        username = self.params["parameter_server"]["ros__parameters"][item.text()]["Username"]
        password = self.params["parameter_server"]["ros__parameters"][item.text()]["Password"]
        print(item.text())
        self.install(host,username, password)
    def install(self, host, username, password):
        
        try:    
            print("Trying to connect")
            SSH_client, SFTP_client = create_clients(host, username, password)
        except Exception as error:
            print(error)
            return
        print("Deleting existing workspace...")
        _stdin, stdout, stderr = SSH_client.exec_command("rm -rf aimotion-f1tenth-system")
        print("Copying workspace onto vehicle...")
        #SFTP_client.rmall("aimotion-f1tenth-system")
        time.sleep(5)
        SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)

        SFTP_client.put_dir(self.path, "aimotion-f1tenth-system")

        SFTP_client.close()


        print("Building ROS workspace...")
        _stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
        for line in iter(stdout.readline, ""):
            print(line, end="")
        if stdout.channel.recv_exit_status():
            print("Failed to build ROS workspace")
        else:
            print("Successfully installed aimotion-f1tenth-system on vehicle")
        SFTP_client.close()
        SSH_client.close()





app = QApplication(sys.argv)
screen = Window()
screen.show()
sys.exit(app.exec_())



#print(path)
"""
for host in IP_list:
    try:    
        SSH_client, SFTP_client = create_clients(host, username, password)
    except Exception as error:
        print(error)
        continue

    print("Copying workspace onto vehicle...")

    SFTP_client.rmall("aimotion-f1tenth-system")
    SFTP_client.mkdir("aimotion-f1tenth-system", ignore_existing=False)

    SFTP_client.put_dir(path, "aimotion-f1tenth-system")

    SFTP_client.close()


    print("Building ROS workspace...")
    _stdin, stdout, stderr = SSH_client.exec_command('bash --login -c "source /opt/ros/foxy/local_setup.bash ;cd aimotion-f1tenth-system/; colcon build"')
    for line in iter(stdout.readline, ""):
        print(line, end="")
    if stdout.channel.recv_exit_status():
        print("Failed to build ROS workspace")
    else:
        print("Successfully installed aimotion-f1tenth-system on vehicle")

    SSH_client.close()

"""

