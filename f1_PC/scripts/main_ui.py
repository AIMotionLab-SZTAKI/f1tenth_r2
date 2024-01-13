from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
from PyQt5.QtGui import *
import sys
import os

import yaml

from PyQt5.QtWidgets import (

    QApplication,

    QPushButton,

    QVBoxLayout,
    QWidget,
    QGridLayout

)


class Window(QWidget):

    def __init__(self):

        super().__init__()

        self.setWindowTitle("Ultimate fleet manager")
        self.resize(270, 110)
        self.vehicle_configs = {}
        main_layout = QGridLayout(self)
        self.setLayout(main_layout)
        ##Variable declaration:
        self.config_path = os.path.join(os.getcwd(), "configs")
        self.params = None
        self.set_config_file()

        ##Widget declaration:

        self.CONFIG_PATH_LABEL = QLabel("config path: " + self.config_path)

        self.CHANGE_CONFIG_PATH_BUTTON = QPushButton("Change")

        self.VEHICLE_LIST =QTableWidget()
        self.VEHICLE_LIST.verticalHeader().setVisible(False)
        self.VEHICLE_LIST.horizontalHeader().setVisible(False)
        self.VEHICLE_LIST.setMaximumWidth(250)
        self.load_vehicle_list()
        #self.VEHICLE_LIST.itemAt(0,0).setBackground(QColor.green)

        ##Addind widgets to layout
        main_layout.addWidget(self.CONFIG_PATH_LABEL, 0,0)
        main_layout.addWidget(self.CHANGE_CONFIG_PATH_BUTTON, 0,1)
        main_layout.addWidget(self.VEHICLE_LIST, 1,0)
        
        ##Connecting Widgets to functions:

        self.CHANGE_CONFIG_PATH_BUTTON.clicked.connect(self.set_config_file)

    def set_config_file(self):
        with open(os.path.join(self.config_path, "param.yaml"), 'r') as config:
            self.params = yaml.load(config, Loader= yaml.FullLoader)
        vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        for v in vehicles:
            self.vehicle_configs[v] = yaml.load(open( self.config_path + "/"+v+ ".yaml", "r"), Loader= yaml.FullLoader)



    def load_vehicle_list(self):
        self.VEHICLE_LIST.setColumnCount(2)
        self.VEHICLE_LIST.setRowCount(1)
        self.VEHICLE_LIST.setItem(0,0,QTableWidgetItem("ID"))
        vehicles = list(self.params["parameter_server"]["ros__parameters"]["vehicle_id_list"])
        row = 0
        self.VEHICLE_LIST.setRowCount(len(vehicles))
        for v in vehicles:
            
            self.vehicle_configs
            self.VEHICLE_LIST.setItem(row,0, QTableWidgetItem(v))
            self.VEHICLE_LIST.setItem(row,1, QTableWidgetItem(str(self.vehicle_configs[v]["parameter_server"]["ros__parameters"]["radio_channel"])))   
            for j in range(self.VEHICLE_LIST.columnCount()):
                self.VEHICLE_LIST.item(row, j).setBackground(Qt.GlobalColor.red)
            row+= 1

if __name__ == "__main__":

    app = QApplication(sys.argv)

    window = Window()

    window.show()

    sys.exit(app.exec_())