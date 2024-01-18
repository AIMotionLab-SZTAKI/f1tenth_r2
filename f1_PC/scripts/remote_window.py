
from PyQt5.QtWidgets import QDoubleSpinBox, QGridLayout, QWidget, QLabel
from PyQt5.QtCore import Qt, QEvent
from PyQt5.QtCore import *

class controller(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QGridLayout()
        self.setWindowModality(Qt.ApplicationModal)
        self.setLayout(self.layout)
        self.installEventFilter(self)
        self.setWindowTitle("Remote Controller")



        #Creating widgets:

        self.DELTA_EDIT = QDoubleSpinBox()
        self.DUTY_EDIT = QDoubleSpinBox()




        self.DELTA_EDIT.setMaximum(0.4)
        self.DELTA_EDIT.setMinimum(0)
        self.DELTA_EDIT.setSingleStep(0.05)
        self.DELTA_EDIT.setValue(0.3)

        self.DUTY_EDIT.setMaximum(0.2)
        self.DUTY_EDIT.setMinimum(0)
        self.DUTY_EDIT.setSingleStep(0.01)
        self.DUTY_EDIT.setValue(0.08)

        self.DELTA_LABEL = QLabel("Delta: ")
        self.DUTY_LABEL = QLabel("Duty cycle: ")

        self.layout.addWidget(self.DELTA_LABEL,0,0)
        self.layout.addWidget(self.DELTA_EDIT,0,1 )

        self.layout.addWidget(self.DUTY_LABEL, 1,0)
        self.layout.addWidget(self.DUTY_EDIT,1,1 )

        


        self.forward = 0.0
        self.rot = 0.0
        self.resize(300,300)
    def eventFilter(self,source, event):
        """Catches keyboard events, modifies the control inputs"""

        # key press
        
        
        if event.type() == QEvent.KeyPress:
            key = event.key()
            if key==Qt.Key_Up:
                self.forward = self.DUTY_EDIT.value()
            elif key==Qt.Key_Down:
                self.forward=- self.DUTY_EDIT.value()
            elif key==Qt.Key_Left:
                self.rot=self.DELTA_EDIT.value()
            elif key==Qt.Key_Right:
                self.rot=-self.DELTA_EDIT.value()
            elif key==Qt.Key_Shift: # return focus instead of quitting
                self.setFocus()
                


        # key release
        if event.type() == QEvent.KeyRelease:
            key = event.key()
            if key==Qt.Key_Up and self.forward>0:
                self.forward=0.0  
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

