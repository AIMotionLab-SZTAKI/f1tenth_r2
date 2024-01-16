from trajectory_msg.srv import Trajectory, Feedback
from rclpy.node import Node






class trajectory_client(Node):
    def __init__(self, vehicle_name):
        super().__init__(vehicle_name+'_trajectory_client')
        self.trajectory_client = self.create_client(Trajectory, vehicle_name+'_execute_trajectory')
        self.progress_srv =  None#self.create_service(Feedback, vehicle_name+"_vehicle_feedback",self.feedback_callback )
        
        

    def send_request(self, path):

        request = Trajectory.Request()
        
        request.path_t = []
        for i in range(len(path.tck[0])):
            request.path_t.append(float(path.tck[0][i]))
        request.path_cx = []
        for i in range(len(path.tck[1][0])):
            request.path_cx.append(float(path.tck[1][0][i]))
        request.path_cy = []
        for i in range(len(path.tck[1][1])):
            request.path_cy.append(float(path.tck[1][1][i]))
        request.path_k = path.tck[2]


        request.speed_t = []
        #print(path.speed_tck)
        for i in range(len(path.speed_tck[0])):
            request.speed_t.append(float(path.speed_tck[0][i]))
        request.speed_c = []
        for i in range(len(path.speed_tck[1])):
            request.speed_c.append(float(path.speed_tck[1][i]))
        request.speed_k = path.speed_tck[2]
        request.s_start = 0.0
        request.s_end = float(path.length)

        try:
            if self.trajectory_client.wait_for_service(timeout_sec= 2) == False:
                raise SystemError("Service is not running")
            self.future = self.trajectory_client.call_async(request)
            print("Goal sent")
        #rclpy.spin_until_future_complete(self, self.future)
        #print(self.future.result().received)
        except Exception as error :
            print(error)