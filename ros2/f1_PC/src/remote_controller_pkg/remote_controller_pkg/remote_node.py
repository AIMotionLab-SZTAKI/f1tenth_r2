import sys
import asyncio
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QShortcut, QSlider, QPushButton
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QKeySequence
##ROS2 importok:
import rclpy
from drive_bridge_msg.msg import InputValues
from mocap_msg.msg import PosValue
from std_msgs.msg import Float64
from rclpy.node import Node
from vehicle_state_msgs.msg import VehicleStateStamped
import threading
import csv


threads = list()
logging = True

""" Example:
with open('employee_file2.csv', mode='w') as csv_file:
    fieldnames = ['emp_name', 'dept', 'birth_month']
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

    writer.writeheader()
    writer.writerow({'emp_name': 'John Smith', 'dept': 'Accounting', 'birth_month': 'November'})
    writer.writerow({'emp_name': 'Erica Meyers', 'dept': 'IT', 'birth_month': 'March'})
"""
class StateNote(Node):
    def __init__(self):
        super().__init__('remote_state_listener')
        self.listener = self.create_subscription(VehicleStateStamped,"/JoeBush1_state", self.state_listener_callback, 1)
        print('state_listener initialised')
        self.logging_status = False
        self.fieldnames = ['time_stamp_sec', 'position_x', 'position_y', 'heading_angle', 'velocity_x', 'velocity_y', 'omega', 'duty_cycle', 'delta', 'erpm']
        self.csv_file =  open('my_log.csv', mode='w')
        self.writer = csv.DictWriter(self.csv_file, fieldnames= self.fieldnames)
        self.writer.writeheader()

    def state_listener_callback(self, data):
        if self.logging_status == True:
            #print(data.heading_angle)
            self.writer.writerow({
                'time_stamp_sec':data.header.stamp.sec + data.header.stamp.nanosec/10**9,
                'position_x': data.position_x,
                'position_y': data.position_y,
                'heading_angle': data.heading_angle,
                'velocity_x': data.velocity_x,
                'velocity_y': data.velocity_y,
                'omega': data.omega,
                'duty_cycle': data.duty_cycle,
                'delta': data.delta,
                'erpm': data.erpm
                  })
class MocapSubscriber(Node):

    def __init__(self):
        super().__init__('mocap_subscriber')
        self.subscription = self.create_subscription(
            PosValue,
            '/mocap',
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.name)


class ControlPublisher(Node):
    def __init__(self):
        super().__init__("control")
        self.command_pub = self.create_publisher(InputValues, "/JoeBush1_control",1)
        print("control node inicializalva")

class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.kerekallas = 0.5

        rclpy.init()
        ##ROS2 node létrehozása
        self.node = ControlPublisher()

        self.window_width, self.window_height = 600, 600
        self.setMinimumSize(self.window_width, self.window_height)
        self.label = QLabel(self)
        self.label.setText('Welcome')
        self.label.move(150, 150)
        #Adding the slider to adjust the max steering 'angle' output
        self.steering_slider = QSlider(Qt.Vertical, self)
        self.steering_slider.setRange(0,100) #Ezt fogom elosztani 100-al, és így max 1 rad lesz a kiadható kerékszög
        self.steering_slider.setTickInterval(1)
        self.steering_slider.setSingleStep(1)
        self.steering_slider.move(100,100)
        self.steering_slider.valueChanged.connect(self.angle_update)
        self.steering_slider.setPageStep(1)
        self.steering_angle = 0.0
        #Adding the slider to adjust the max speed output
        self.speed_slider = QSlider(Qt.Vertical, self)
        self.speed_slider.setRange(0,30)
        self.speed_slider.move(300,100)
        self.speed_slider.setPageStep(1)
        self.speed_slider.valueChanged.connect(self.speed_update)
        self.speed = 0.0
        ##Registering key strokes
        QShortcut(QKeySequence(Qt.Key_Left), self, activated=self.move_left)
        QShortcut(QKeySequence(Qt.Key_Right), self, activated=self.move_right)
        QShortcut(QKeySequence(Qt.Key_Up), self, activated=self.move_up)
        QShortcut(QKeySequence(Qt.Key_Down), self, activated=self.move_down)
        #creating the button that starts logging
        start_button = QPushButton(self, text = 'start')
        start_button.move(500,500)
        start_button.clicked.connect(self.button_start_logging)
        stop_button = QPushButton(self, text = 'stop')
        stop_button.move(900,500)
        stop_button.clicked.connect(self.button_stop_logging)

        self.my_node = StateNote()
        self.x = threading.Thread(target= rclpy.spin, args=[self.my_node])
        threads.append(self.x)
        self.x.start()

    def button_start_logging(self):
        self.my_node.logging_status = True
        print("logging started")
    def button_stop_logging(self):
        self.my_node.logging_status = False
        print("logging stopped")

    def angle_update(self, value):
        self.steering_angle = value/100
        print('steering angle set to:',self.steering_angle)
    def speed_update(self, value):
        self.speed = value/100
        print('max speed set to:',self.speed)
    def move_left(self):

        if self.kerekallas < 0:
            self.label.setText("egyenes")
            self.kerekallas = 0.0
            msg = InputValues()
            msg.delta = 0.0
            msg.d = 0.0
        else:
            self.label.setText("balra")
            self.kerekallas = +self.steering_angle
            msg = InputValues()
            msg.delta = float(+self.steering_angle)
            msg.d = 0.0
        self.node.command_pub.publish(msg)
        print("Pusblishing: ", msg.delta, " ", msg.d )

    def move_right(self):

        if self.kerekallas > 0:
            self.label.setText("egyenes")
            self.kerekallas = 0.0
            msg = InputValues()
            msg.delta = 0.0
            msg.d = 0.0
        else:
            self.label.setText("jobbra")
            self.kerekallas = -self.steering_angle
            msg = InputValues()
            msg.delta = float(-self.steering_angle)
            msg.d = 0.0
        self.node.command_pub.publish(msg)
        print("Puslishing: ", msg.delta, " ", msg.d )

    def move_up(self):
        self.label.setText("elore")
        msg = InputValues()
        msg.delta = float(self.kerekallas)
        msg.d = self.speed
        self.node.command_pub.publish(msg)
        print("Puslishing: ", msg.delta, " ", msg.d )
        ##self.node.get_logger().info('Publishing: "%s"' % msg.delta)

    def move_down(self):
        self.label.setText("hatra")
        msg = InputValues()
        msg.delta = float(self.kerekallas)
        msg.d = -self.speed
        self.node.command_pub.publish(msg)
        print("Puslishing: ", msg.delta, " ", msg.d )
def main():
        app = QApplication(sys.argv)
        app.setStyleSheet('''
            QWidget {
                font-size: 45px;
            }
        ''')

        myApp = MyApp()
        main = threading.Thread(target= myApp.show)
        threads.append(main)
        main.start()
        
        try:
            sys.exit(app.exec_())
        except SystemExit:
            print('Closing Window...')
            rclpy.shutdown()
            myApp.my_node.csv_file.close()
        
if __name__ == '__main__':
    # don't auto scale when drag app to a different monitor.
    # QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    main()
