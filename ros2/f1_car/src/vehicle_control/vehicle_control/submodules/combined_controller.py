import numpy as np
import rclpy
from scipy.interpolate import splev
from vehicle_state_msgs.msg import VehicleStateStamped
from drive_bridge_msg.msg import InputValues
import math
from os.path import expanduser
import time
from trajectory_msg.srv import Trajectory, Feedback
#from rclpy.action import ActionServer
from rclpy.node import Node
from .control_util import BaseController, project_to_closest, _normalize, _clamp
from std_msgs.msg import Float64

class CombinedController(BaseController):
    def __init__(self, FREQUENCY, lateral_gains, longitudinal_gains, projection_window, projection_step, vehicle_id, vehicle_params, look_ahead=0):
        super(CombinedController, self).__init__(FREQUENCY=FREQUENCY, car_id= vehicle_id)
        
        # required params for feedback control
        self.projection_window=projection_window
        self.projection_step=projection_step
        
        self.car_id = vehicle_id
        # Lateral dynamics integral term
        self.q=0

        # look ahead distance for evaluation
        self.look_ahead=look_ahead

        # feedback gain polynomials
        self.k_lat1=np.poly1d(lateral_gains["k1"])
        self.k_lat2=np.poly1d(lateral_gains["k2"])
        self.k_lat3=np.poly1d(lateral_gains["k3"])

        
        self.k_lat1_r=np.poly1d(lateral_gains["k1_r"])
        self.k_lat2_r=np.poly1d(lateral_gains["k2_r"])
        #self.k_lat1_r=np.poly1d([-0.0008,0.0442, -1.2247])
        #self.k_lat2_r=np.poly1d([-0.0002,0.0191,-0.9531])

        self.k_long1=np.poly1d(longitudinal_gains["k1"])
        self.k_long2=np.poly1d(longitudinal_gains["k2"])
        #self.m=longitudinal_gains["m"]



        self.l_offs=.2

        self.m=vehicle_params["m"]
        self.C_f=vehicle_params["C_f"]
        self.C_r=vehicle_params["C_r"]
        self.l_f=vehicle_params["l_f"]
        self.l_r=vehicle_params["l_r"]
        self.C_m1=vehicle_params["C_m1"]
        self.C_m2=vehicle_params["C_m2"]
        self.C_m3=vehicle_params["C_m3"]

        """
        self.logfile=open(expanduser("~")+"/car1.csv", "w")
        self.logfile.write(str(self.k_long1(1))+"\n")
        
        self.logfile.write("t,x,y,phi,v_xi,v_eta,omega,d,delta,z1,theta_e,s_err,v_err\n")
        """
        
        self.get_logger().info("LPV-LQR trajectory-tacking controller initialized!")

    def execute_trajectory(self, trajectory_request, response):
        self.get_logger().info("Trajectory execution request recieved!")
        super(CombinedController,self).execute_trajectory(trajectory_request, response)
        self.q=0
        ##TODO get rid of this, we don't need to overwrite the inherited function
        #self.action_result.success = True
        response.received = True
        return response

    
    def state_callback(self, data):
        """
        Callback function triggered by the state subscriber that calculates the control inputs of the system.
        """
        
        super(CombinedController,self).state_callback(data)
        

        ### CHECK IF THE CONTROLLER IS ENABLED
        if not self.enabled:
            return
        

        ##Getting time stamp: Correct???
        
        time = self.get_time()

        ### PROCESS STATE DATA ###    
        # get velocity data
        v_xi=data.velocity_x
        v_eta=data.velocity_y

        # get the current position
        position=np.array([ 
                            data.position_x+np.sign(v_xi)*self.look_ahead*np.cos(data.heading_angle),
                            data.position_y+np.sign(v_xi)*self.look_ahead*np.sin(data.heading_angle)
                            ]
                            ) # adaptive look ahead to reduce the effect of time delay
        
        if v_xi<0:
            position=np.array([ 
                            data.position_x-self.l_offs*np.cos(data.heading_angle),
                            data.position_y-self.l_offs*np.sin(data.heading_angle)]) 
    

        phi=_normalize(data.heading_angle)

        # CoM side-slip
        beta=np.arctan2(v_eta,abs(v_xi)) # abs() to condiser backward motion ass well

        ### PROJECT ONTO PATH###
        # estimate path parameter velocity
        #s_est=self.s+abs(v_xi)*self.dt
        
        s_ref = splev(time, self.evol_tck)

        # project
        self.s=project_to_closest(pos=position,s_est=s_ref, s_window=self.projection_window,
                            path=self.get_path,step=self.projection_step, s_bounds=(self.s_start,self.s_end))
        
        
        # get path data at projected reference
        ref_pos, s0, z0, v_ref, c=self.get_path_data(self.s)

        if pow(position[0]-ref_pos[0],2)+ pow(position[1]-ref_pos[1],2) > pow(5,5):#pow(1,2):
            #print("x error: ", position[0]-ref_pos[0], " y error: ", position[1]-ref_pos[1])
            #print("self shutdown")
            self.shutdown()
        # invert heading in case of backward motion
        if v_ref<0:
            phi+=np.pi


        ### CHECK CURRENT POSITION & PROVIDE ROS ACTION FEEDBACK ###
        if self.check_progress():
            return # if the goal is reached exit the function


        ### MODEL STATES IN PATH COORDINATES
        # path tangent angle 
        theta_p = np.arctan2(s0[1], s0[0])



        # lateral error
        z1=np.dot(position-ref_pos, z0)
        """
        if z1 > 0.2:
            print("Lateral error-> self shutdown")
            self.shutdown()
        """
        # heading error
        theta_e=_normalize(phi-theta_p)

        # longitudinal model parameter
        p=abs(np.cos(theta_e+beta)/np.cos(beta)/(1-c*z1))


        # invert z1 for lateral dynamics:
        e=-z1
        self.q+=e
        self.q=_clamp(self.q,0.1)

        # estimate error derivative
        try:
            self.edot=0.5*((e-self.ep)/self.dt-self.edot)+self.edot
            self.ep=e
        except AttributeError:
            self.edot=0
            self.ep=e

        ### FEEDBACK CONTROL ###
        # lateral
        k_lat1,k_lat2,k_lat3=self.get_lateral_feedback_gains(v_xi, v_ref)

        if v_ref>0:
            delta=theta_e-k_lat1*self.q-k_lat2*e-k_lat3*self.edot - self.m/self.C_f*((self.l_r*self.C_r-self.l_f*self.C_f)/self.m-1)*c

        elif v_ref<0:
            delta=1.5*k_lat1*z1+k_lat2*theta_e#+0.33/((abs(v_xi)+0.01)*c*np.cos(theta_e)/(1-c*z1)) 
            #delta=0.33/((abs(v_xi)+0.01)*c*np.cos(theta_e)/(1-c*z1))
        
        else:
            delta=0
                
        #longitudinal
        k_long1,k_long2=self.get_longitudinal_feedback_gains(p)
        #d=-k_long1*(self.s-self.s_ref)-k_long2*(v_xi-v_ref)
        if v_ref>0:
            d=(self.C_m2*v_ref+self.C_m3*np.sign(v_ref))/self.C_m1-k_long1*(self.s-s_ref)-k_long2*(v_xi-v_ref)
            if d<0: d=0
        else:
            d=(self.C_m2*v_ref+self.C_m3*np.sign(v_ref))/self.C_m1+k_long1*(self.s-s_ref)-k_long2*(v_xi-v_ref)
            if d>0: d=0
        
        
        ### PUBLISH INPUTS ###
        msg=InputValues()
      #  msg.d= Float64()
        if d > 0.2:
            d = 0.2
        if d < -0.15:
            d = -0.15
            
        msg.d = float(d) ##!!!!!!!!!!!!!!!!!! set d
     #   msg.delta=Float64()
        msg.delta = float(-delta)

        self.pub.publish(msg)

    def get_lateral_feedback_gains(self, v_xi, v_ref):
        if v_ref>0:
            k1=self.k_lat1(v_xi)
            k2=self.k_lat2(v_xi)
            k3=self.k_lat3(v_xi)

        else:
            k1=self.k_lat1_r(v_xi)
            k2=self.k_lat2_r(v_xi)
            k3=None
        return k1,k2,k3

        
    def get_longitudinal_feedback_gains(self,p):
        k1=self.k_long1(p)
        k2=self.k_long2(p)
        return k1, k2