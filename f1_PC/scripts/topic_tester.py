import rclpy
from rclpy.node import Node
from vehicle_state_msgs.msg import VehicleStateStamped
import matplotlib.pyplot as plt
import numpy 
import time

plt.ion()
fig, ax = plt.subplots()
start = time.time()

lines, = ax.plot([],[], "g")
class MyNode(Node):
    def __init__(self):
        super().__init__("tester_node")
        self.client = self.create_subscription(VehicleStateStamped, "/state",self.cucc_callback, 1 )
        self.n = 0
    def cucc_callback(self, data):
        current_time = time.time()-start
        self.n+= 1
        """
        lines.set_ydata(numpy.append(lines.get_ydata(), current_time))
        lines.set_xdata(numpy.append(lines.get_xdata(), self.n))
        """
        
        data_time = float(str(data.header.stamp.sec)+ "."+str(data.header.stamp.nanosec).zfill(9))
        d_time = data_time-time.time()
        lines.set_xdata(numpy.append(lines.get_xdata(), current_time))
        lines.set_ydata(numpy.append(lines.get_ydata(), d_time))
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()  

        pass

rclpy.init()
tester = MyNode()
rclpy.spin(tester)
