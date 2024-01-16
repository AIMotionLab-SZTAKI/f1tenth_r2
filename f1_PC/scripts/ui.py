import paramiko
import os
import install_utils 
from install_utils import create_clients
import time
from pathlib import Path
import yaml
import PyQt5
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QSizePolicy
from PyQt5 import QtGui
import sys
from Parameter_editor import *
from installer_ui import *
from param_edit_window import *
import shutil
from login_info import Login_info_window
class MainWindow(QWidget):
    def update_listbox(self):
        q = 0
        self.LISTWIDGET.clear()
        vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        if "Template" in vehicles:
            vehicles.remove("Template")
        for i in vehicles:
            self.LISTWIDGET.insertItem(q, i)
            q+=1
    def __init__(self):
        super().__init__()
        self.setWindowIcon(PyQt5.QtGui.QIcon("icon.jpeg"))
        config = os.path.join("configs",  'param.yaml')
        config = open(config, 'r')
        self.params = yaml.load(config, Loader= yaml.FullLoader)
        QWidget.__init__(self)
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.setWindowTitle("Fleet Manager")
        self.setMinimumWidth(600)
        self.setMinimumHeight(600)
        self.LISTWIDGET = QListWidget()
        self.LISTWIDGET.setMaximumWidth(150)
        self.PARAM_EDIT_BUTTON = QPushButton()
        self.ADD_VEHICLE_BUTTON = QPushButton(text = "Add new vehicle")
        self.REMOVE_VEHICLE_BUTTON = QPushButton(text= "Remove vehicle")
        self.INSTALL_BUTTON = QPushButton(text = "Install")
        self.LOG_EDIT_BUTTON = QPushButton(text= "Login edit")
        self.OK = QPushButton(text= "OK")
        self.CANCEL = QPushButton(text= "CANCEL")

        self.INSTALL_BUTTON.setFixedWidth(120)
        self.PARAM_EDIT_BUTTON.setFixedWidth(120)
        self.REMOVE_VEHICLE_BUTTON.setFixedWidth(120)
        self.ADD_VEHICLE_BUTTON.setFixedWidth(120)
        self.LOG_EDIT_BUTTON.setFixedWidth(120)
        
        self.TEXTBOX = QTextEdit()
        self.INSTALL_BUTTON.clicked.connect(self.install_button_event)
        self.update_listbox()
        self.PARAM_EDIT_BUTTON.setText("Edit parameters")
        self.PARAM_EDIT_BUTTON.clicked.connect(self.param_edit)
        self.LISTWIDGET.clicked.connect(self.item_select_event)
        self.ADD_VEHICLE_BUTTON.clicked.connect(self.new_vehicle)
        self.REMOVE_VEHICLE_BUTTON.clicked.connect(self.Remove_vehicle)
        self.LOG_EDIT_BUTTON.clicked.connect(self.login_edit)
        self.param_edit = None  
        self.installer = None
        self.log_edit = None
        self.selected_item = None
        self.layout.addWidget(self.LISTWIDGET, 0,0)
        self.layout.addWidget(self.INSTALL_BUTTON, 2, 2, alignment=QtCore.Qt.AlignRight)
        self.layout.addWidget(self.PARAM_EDIT_BUTTON, 2, 0,alignment= QtCore.Qt.AlignLeft )
        self.layout.addWidget(self.ADD_VEHICLE_BUTTON, 3, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3, 2, alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.LOG_EDIT_BUTTON, 2,1 ,alignment= QtCore.Qt.AlignCenter)
    def login_edit(self, checked):
        if self.selected_item is None:
            print("Select a vehicle")
            return
        self.log_edit = Login_info_window(self.selected_item)
        self.log_edit.show()
    def param_edit(self, checked):
        if self.selected_item is None:
            print("Select a vehicle")
            return
        self.param_edit = Param_editor_window(self.selected_item)
        self.param_edit.show()
    def item_select_event(self, qmodelindex):
        self.selected_item = self.LISTWIDGET.currentItem().text()
    def install_button_event(self):
        if self.selected_item is None:
            print("Select a vehicle")
            return
        self.installer = Installer_window(self.selected_item)
    def new_vehicle(self):
        self.REMOVE_VEHICLE_BUTTON.setParent(None)
        self.INSTALL_BUTTON.setParent(None)
        self.ADD_VEHICLE_BUTTON.setParent(None)
        self.PARAM_EDIT_BUTTON.setParent(None)
        self.LOG_EDIT_BUTTON.setParent(None)
        self.layout.addWidget(self.OK, 2, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.CANCEL, 2, 3,alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.TEXTBOX, 1, 2, alignment= QtCore.Qt.AlignLeft)
        self.TEXTBOX.setMaximumHeight(20)
        self.OK.clicked.connect(self.OK_NEW_VEHICLE)
        self.CANCEL.clicked.connect(self.CANCEL_NEW_VEHICLE)
    def OK_NEW_VEHICLE(self):
        print("OK NEW VEHICLE")
        current_vehicles =list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        new = str(self.TEXTBOX.toPlainText())
        current_vehicles.append(new)
        shutil.copy(os.path.join(os.getcwd(), "configs", "Template.yaml"), os.path.join(os.getcwd(), "configs",  new +".yaml"))
        shutil.copy(os.path.join(os.getcwd(), "configs", "Template_login.yaml"), os.path.join(os.getcwd(), "configs",  new +"_login.yaml"))

        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None)
        self.layout.addWidget(self.INSTALL_BUTTON, 2, 1, alignment=QtCore.Qt.AlignRight)
        self.layout.addWidget(self.PARAM_EDIT_BUTTON, 2, 0,alignment= QtCore.Qt.AlignLeft )
        self.layout.addWidget(self.ADD_VEHICLE_BUTTON, 3, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3, 1, alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.LOG_EDIT_BUTTON, 2,1 ,alignment= QtCore.Qt.AlignCenter)

        self.OK.clicked.disconnect()
        self.CANCEL.clicked.disconnect()
        self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"] =  current_vehicles
        with open(os.path.join("configs", "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()
        with open("configs/"+new +".yaml", "r") as file:
            t_params = yaml.load(file, Loader=yaml.FullLoader)
            t_params["parameter_server"]["ros__parameters"]["car_id"] = str(new)
            
            file.close()
            saver_file =  open("configs/"+new +".yaml", "w")
            yaml.dump(t_params, saver_file, default_flow_style=False)
            saver_file.close()
        self.TEXTBOX.clear()
        self.update_listbox()
    def CANCEL_NEW_VEHICLE(self):
        print("CANCEL NEW VEHICLE")
        self.TEXTBOX.setText("")
        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.TEXTBOX.setParent(None)
        self.layout.addWidget(self.INSTALL_BUTTON, 2, 1, alignment=QtCore.Qt.AlignRight)
        self.layout.addWidget(self.PARAM_EDIT_BUTTON, 2, 0,alignment= QtCore.Qt.AlignLeft )
        self.layout.addWidget(self.ADD_VEHICLE_BUTTON, 3, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3, 1, alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.LOG_EDIT_BUTTON, 2,1 ,alignment= QtCore.Qt.AlignCenter)

        self.OK.clicked.disconnect()
        self.CANCEL.clicked.disconnect()
    def Remove_vehicle(self):
        if self.selected_item is None:
            print("Select a vehicle")
            return
        self.REMOVE_VEHICLE_BUTTON.setParent(None)
        self.INSTALL_BUTTON.setParent(None)
        self.ADD_VEHICLE_BUTTON.setParent(None)
        self.PARAM_EDIT_BUTTON.setParent(None)
        self.LOG_EDIT_BUTTON.setParent(None)
        
        self.layout.addWidget(self.OK, 2, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.CANCEL, 2, 1,alignment= QtCore.Qt.AlignRight)
        self.OK.clicked.connect(self.OK_REMOVE)
        self.CANCEL.clicked.connect(self.CANCEL_REMOVE)
    def OK_REMOVE(self):
        print("OK_REMOVE")
        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.layout.addWidget(self.INSTALL_BUTTON, 2, 1, alignment=QtCore.Qt.AlignRight)
        self.layout.addWidget(self.PARAM_EDIT_BUTTON, 2, 0,alignment= QtCore.Qt.AlignLeft )
        self.layout.addWidget(self.ADD_VEHICLE_BUTTON, 3, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3, 1, alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.LOG_EDIT_BUTTON, 2,1 ,alignment= QtCore.Qt.AlignCenter)

        currentvehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        currentvehicles.remove(self.selected_item)
        self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"] = currentvehicles
        with open(os.path.join("configs", "param.yaml"), "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()
        self.update_listbox()
        self.OK.clicked.disconnect()
        self.CANCEL.clicked.disconnect()
        try:
            os.remove(os.path.join("configs", self.selected_item + ".yaml"))
            os.remove(os.path.join("configs", self.selected_item + "_login.yaml"))
        except(Exception):
            print("Error removing the yaml files")
        
        return
    def CANCEL_REMOVE(self):
        print("CANCEL REMOVE")
        self.OK.setParent(None)
        self.CANCEL.setParent(None)
        self.layout.addWidget(self.INSTALL_BUTTON, 2, 1, alignment=QtCore.Qt.AlignRight)
        self.layout.addWidget(self.PARAM_EDIT_BUTTON, 2, 0,alignment= QtCore.Qt.AlignLeft )
        self.layout.addWidget(self.ADD_VEHICLE_BUTTON, 3, 0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.REMOVE_VEHICLE_BUTTON, 3, 1, alignment= QtCore.Qt.AlignRight)
        self.layout.addWidget(self.LOG_EDIT_BUTTON, 2,1 ,alignment= QtCore.Qt.AlignCenter)


        self.OK.clicked.disconnect()
        self.CANCEL.clicked.disconnect()
        return
app = QApplication(sys.argv)
screen = MainWindow()
screen.show()
sys.exit(app.exec_())