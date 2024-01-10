from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import *
import os
import yaml
class Login_info_window(QWidget):
    def __init__(self,vehicle_name):
        self.vehicle_name = vehicle_name
        with open("configs/" +vehicle_name + "_login.yaml", "r") as file:
            self.params = yaml.load(file, Loader=yaml.FullLoader)
            file.close()
        super().__init__()
        QWidget.__init__(self)
        self.layout = QGridLayout()
        
        self.setLayout(self.layout)
        self.label1 = QLabel(text= "Hostname:")
        self.textbox1 = QTextEdit()
        self.textbox1.setText(str(self.params["IP"]))
        self.label2 = QLabel(text= "Username")
        self.textbox2 = QTextEdit()
        self.textbox2.setText(str(self.params["Username"]))
        self.label3 = QLabel(text= "Password")
        self.textbox3 = QTextEdit()
        self.textbox3.setText(str(self.params["Password"]))

        
        self.OK_BUTTON = QPushButton(text= "OK")
        self.CANCEL_BUTTON = QPushButton(text= "CANCEL")

        self.textbox1.setMaximumHeight(20)
        self.textbox2.setMaximumHeight(20)
        self.textbox3.setMaximumHeight(20)

        self.layout.addWidget(self.label1, 0,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.label2, 2,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.label3, 4,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.textbox1, 1,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.textbox2, 3,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.textbox3, 5,0,alignment= QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.OK_BUTTON, 6,0, alignment= QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.CANCEL_BUTTON, 6,0, alignment= QtCore.Qt.AlignRight)

        self.CANCEL_BUTTON.clicked.connect(self.close)
        self.OK_BUTTON.clicked.connect(self.save)
    def save(self):
        self.params["IP"] = self.textbox1.toPlainText()
        self.params["Username"] = self.textbox2.toPlainText()
        self.params["Password"] = self.textbox3.toPlainText()
        with open("configs/" + self.vehicle_name+ "_login.yaml", "w") as file:
            yaml.dump(self.params, file, default_flow_style= False)
            file.close()
            self.close()
