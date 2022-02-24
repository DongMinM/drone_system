#!/usr/bin/python3
import numpy as np
import rospy
import scipy.sparse as ssp
import scipy.sparse.linalg as sla
from trajectory_velo_generation import Trajectory_Velocity_Generator

from drone_system.msg import Status
from std_msgs.msg import String,Float32MultiArray

class TrajectoryGenerator:

    def __init__(self):
        
        rospy.init_node("trajectory_generator")
        rospy.Subscriber("action_msgs", String, self.action_update)
        self.pub2motion_motion = rospy.Publisher("motion_msgs", String,queue_size=1)
        self.pub2motion_trajec = rospy.Publisher("trajec_msgs", Float32MultiArray,queue_size=1)

        self.action = None
        self.x_0 = np.array([0,0,0,0,0,0])                                            # need input
        self.x_des = np.array([0,0,0,0,0,0])                                          # need input

        self.distance = 0
        self.Time = 0
        self.n = 0

        self.gen_number = 0                                                           # slice generation number


    def action_update(self,action_msgs):

        self.action = action_msgs.data

    def update_cur_status(self,data_hub):            ## need memory connection
        self.x_0 = np.array([data_hub.pos_n,data_hub.pos_e,data_hub.pos_d,data_hub.vel_n,data_hub.vel_e,data_hub.vel_d])

    def update_distance(self,x_des):
        self.x_des = x_des
        self.distance = np.sum((self.x_des[0:3]-self.x_0[0:3])**2)**0.5

        if self.gen_number == 0:                     ## set total time

                self.Time = round(self.distance*2)
                self.n = self.Time*10

                if self.Time <= 0.4:                                                          
                    self.Time = 0.4  
                    self.n = self.Time*10


    def run(self):
        while not rospy.is_shutdown():
            if self.action != None:

                if self.action == "arm":
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    self.action = None

                if self.action == "take_off":

                    # destination_param = np.array([0,0,self.action[1],0,0,0])
                    destination_param = np.array([0,0,5,0,0,0])
                    
                    self.update_distance(destination_param)
                    trajectory_velocity = Trajectory_Velocity_Generator().run(self)           # [  v0,  v1,  v2,  v3  ] 

                    self.pub2motion_motion.publish(self.action)
                    velocity_list = Float32MultiArray()
                    velocity_list.data = trajectory_velocity       
                    self.pub2motion_trajec.publish(velocity_list)
                    
                    # self.action = None

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass
    

if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()