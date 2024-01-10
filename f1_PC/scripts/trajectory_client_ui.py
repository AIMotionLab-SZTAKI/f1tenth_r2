import sys
import PyQt5
from PyQt5.QtWidgets import *
import os
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msg.srv import Trajectory, Feedback
import numpy as np
from scipy.interpolate import splev, splprep, splrep
import matplotlib.pyplot as plt
from std_msgs.msg import Float64MultiArray, Float64
from vehicle_state_msgs.msg import VehicleStateStamped
from Path import Path

class MainWindow(QWidget):
    def update_listbox(self):
        files = [f for f in os.listdir('trajectories/') if str(f).__contains__(".npy")]
        print(files)
        q = 0
        self.LISTWIDGET.clear()
        for f in files:
            self.LISTWIDGET.insertItem(q, f)
            q +=1
    def item_select_event(self, qmodelindex):
        self.ros_client.selected_trajectory = self.LISTWIDGET.currentItem().text()
        print(self.ros_client.selected_trajectory)
    def open_trajectory(self):
        if self.ros_client.selected_trajectory == None:
            print("Select a trajectory")
            print(self.ros_client.selected_trajectory)
            return
        print("trajectories/"+ self.ros_client.selected_trajectory)
        data = np.load("trajectories/"+ self.ros_client.selected_trajectory)
        path = Path(data)
        path.show()
   

    def __init__(self):
        super().__init__()
        QWidget.__init__(self)
        rclpy.init()
        self.ros_client = trajectory_client()
        self.layout = QGridLayout()
        self.setLayout(self.layout)
        self.setWindowTitle("Trajectory Client")
        self.setMinimumWidth(600)
        self.setMinimumHeight(600)


        self.LISTWIDGET = QListWidget()
        self.LISTWIDGET.setMaximumWidth(150)
        self.LISTWIDGET.clicked.connect(self.item_select_event)
        self.layout.addWidget(self.LISTWIDGET, 0,0)
        self.update_listbox()
        
        self.SHOW = QPushButton(text = "Show")
        self.SHOW.setMaximumWidth(100)
        self.layout.addWidget(self.SHOW, 1, 1)
        self.SHOW.clicked.connect(self.open_trajectory)

        self.SEND = QPushButton(text = "Send")
        self.SEND.setMaximumWidth(100)
        self.layout.addWidget(self.SEND, 1,2)
        self.SEND.clicked.connect(self.ros_client.send_request)


class trajectory_client(Node):
    def __init__(self):
        self.selected_trajectory = None
        super().__init__('trajectory_client')
        self.trajectory_client = self.create_client(Trajectory, '/execute_trajectory')
        self.progress_srv = self.create_service(Feedback, "vehicle_feedback",self.feedback_callback )

    def send_request(self):
        if self.selected_trajectory == None:
            return
        msg_goal = Trajectory.Request()
        data = np.load("trajectories/"+ self.selected_trajectory)
        path = Path(data)
        msg_goal.path_t = []
        for i in range(len(path.tck[0])):
            msg_goal.path_t.append(float(path.tck[0][i]))
        msg_goal.path_cx = []
        for i in range(len(path.tck[1][0])):
            msg_goal.path_cx.append(float(path.tck[1][0][i]))
        msg_goal.path_cy = []
        for i in range(len(path.tck[1][1])):
            msg_goal.path_cy.append(float(path.tck[1][1][i]))
        msg_goal.path_k = path.tck[2]


        msg_goal.speed_t = []
        #print(path.speed_tck)
        for i in range(len(path.speed_tck[0])):
            msg_goal.speed_t.append(float(path.speed_tck[0][i]))
        msg_goal.speed_c = []
        for i in range(len(path.speed_tck[1])):
            msg_goal.speed_c.append(float(path.speed_tck[1][i]))
        msg_goal.speed_k = path.speed_tck[2]
        msg_goal.s_start = 0.0
        msg_goal.s_end = float(path.length)
        self.trajectory_client.wait_for_service()
        self.future = self.trajectory_client.call_async(msg_goal)
        print("Goal sent")
        rclpy.spin_until_future_complete(self, self.future)
        print(self.future.result().received)


    def feedback_callback(self, request, response):
        print(request.car_id, " ", request.progress, " ", request.succeeded)
        if request.succeeded:
            print(request.car_id, " has finished trajectory")
        response.received = True
        
        return response




def main():
    app = QApplication(sys.argv)
    screen = MainWindow()
    screen.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()    
