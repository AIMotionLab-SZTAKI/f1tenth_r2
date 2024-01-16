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

class Parameter_editor(QWidget):
    def __init__(self, vehicle_name):
        super().__init__()
        layout = QVBoxLayout()
        config = os.path.join("configs",  vehicle_name+".yaml")
        config = open(config, 'r')
        self.dir = [] #this will be used to track where we are currently in the parameter tree
        self.params = yaml.load(config, Loader= yaml.FullLoader)
        self.label = QLabel(vehicle_name)
        self.variable_listbox = QListWidget()
        self.variable_listbox.doubleClicked.connect(self.listbox_select_event)
        self.back_button = QPushButton()
        self.back_button.clicked.connect(self.back_button_push_event)
        self.back_button.setText("Back")
        layout.addWidget(self.label)
        layout.addWidget(self.variable_listbox)
        layout.addWidget(self.back_button)
        q = 0
        for i in self.params["parameter_server"]["ros__parameters"]:
            self.variable_listbox.insertItem(q, i)
            print(i)
            q+=1
        self.setLayout(layout)
        print("opening new window")
    def back_button_push_event(self):
        self.dir.pop()
        print(self.dir)
    def listbox_select_event(self, item):
        self.dir.append(self.variable_listbox.currentItem().text())
        print(self.dir)
    def update_variable_listbox(self):
        self.variable_listbox.clear()
        
