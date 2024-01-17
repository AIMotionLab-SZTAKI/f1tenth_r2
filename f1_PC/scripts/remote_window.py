
from PyQt5.QtWidgets import QDialog, QPushButton, QVBoxLayout, QHBoxLayout, QCheckBox, QApplication, QWidget, QLabel, QLineEdit, QComboBox
from PyQt5.QtCore import pyqtSignal, Qt, QEvent, QTimer
from PyQt5.QtGui import QDoubleValidator
import sys

class controller(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()
        self.setLayout(layout)
        self.installEventFilter(self)
        self.setWindowTitle("Remote Controller")

        self.forward = 0
        self.rot = 0
        self.resize(300,300)
    def eventFilter(self,source, event):
        """Catches keyboard events, modifies the control inputs"""

        # key press
        
        
        if event.type() == QEvent.KeyPress:
            key = event.key()
            if key==Qt.Key_Up:
                self.forward = 1
            elif key==Qt.Key_Down:
                self.forward=-1
            elif key==Qt.Key_Left:
                self.rot=1
            elif key==Qt.Key_Right:
                self.rot=-1
            elif key==Qt.Key_Shift: # return focus instead of quitting
                self.setFocus()
                


        # key release
        if event.type() == QEvent.KeyRelease:
            key = event.key()
            if key==Qt.Key_Up and self.forward>0:
                self.forward=0  
            elif key==Qt.Key_Down and self.forward<0:
                self.forward=0.0
            elif key==Qt.Key_Left and self.rot>0:
                self.rot=0.0
            elif key==Qt.Key_Right and self.rot<0:
                self.rot=0.0
            elif key==Qt.Key_Shift: # return focus instead of quitting
                self.setFocus()
        # call parent function
        return super(controller, self).eventFilter(source, event)

