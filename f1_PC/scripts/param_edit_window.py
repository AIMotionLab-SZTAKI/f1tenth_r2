import paramiko
import os
import install_utils 
from install_utils import create_clients
import time
from pathlib import Path
import yaml
import PyQt5
from PyQt5.QtWidgets import *
import sys
import os


class Param_editor_window(QWidget):
    def __init__(self, vehicle_name):
        super().__init__()
        QWidget.__init__(self)
        self.text_viewer = QTextEdit()
        self.setWindowTitle(vehicle_name + " parameters")
        layout = QGridLayout()
        self.vehicle_name = vehicle_name
        self.setLayout(layout)
        file = open(os.path.join(os.getcwd(), "configs", vehicle_name+".yaml"), 'r')
        data = yaml.safe_load(file)
        self.text_viewer.setText(str(yaml.dump(data)))
        self.setMinimumWidth(500)
        self.save_button = QPushButton()
        self.cancel_button = QPushButton()
        self.save_button.setText("Save")
        self.cancel_button.setText("Cancel")
        layout.addWidget(self.text_viewer)
        layout.addWidget(self.cancel_button)
        layout.addWidget(self.save_button)
        self.save_button.clicked.connect(self.save)
        self.cancel_button.clicked.connect(self.cancel)
        file.close()
        self.show()
    def cancel(self):
        self.close()
    def save(self):
        text = self.text_viewer.toPlainText()
        file = open(os.path.join(os.getcwd(), "configs", self.vehicle_name+".yaml"), 'w')
        file.write(text)
        file.close()
        self.close()
        