#!/usr/bin/python3
import numpy as np
import rospy

from drone_system.msg import Status
from std_msgs.msg import Bool,String,Float32MultiArray
from trajectory_velocity_generation import Trajectory_Velocity_Generator


class TrajectoryGenerator:

    def __init__(self):
        
        self.path = Trajectory_Velocity_Generator()       

        rospy.init_node("trajectory_generator")
        rospy.Subscriber("/action_msgs", Float32MultiArray, self.action_update)
        rospy.Subscriber("/sensor_msgs", Status, self.current_status)
        rospy.Subscriber("/trajec_request", Bool, self.action_update)
        self.pub2motion = rospy.Publisher("/motion_msgs", Float32MultiArray,queue_size=1)
        
        self.action = None

        self.current_status = None
        self.x_des = np.array([0,0,0,0,0,0])                                        

        self.distance = 0
        self.Time = 0
        self.n = 0

        self.path_number = 0   



    def current_status(self,status):

        self.current_status = np.array([status.data.pos_n,
                                        status.data.pos_e,
                                        status.data.pos_d,
                                        status.data.vel_n,
                                        status.data.vel_e,
                                        status.data.vel_d])

    def action_update(self,action_msgs):

        self.action = action_msgs.data

    def update_distance(self,x_des):

        self.x_des = x_des
        self.distance = np.sum((self.x_des[0:3]-self.current_status[0:3])**2)**0.5

        if self.path_number == 0:                     ## set total time

                self.Time = round(self.distance*2)
                self.n = self.Time*10

                if self.Time <= 0.4:                                                          
                    self.Time = 0.4  
                    self.n = self.Time*10



    def run(self):


        while not rospy.is_shutdown():

            if self.action != None:

                if self.action[0] in [0,1,3]:
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None

                elif self.action[0] == 2: # take_off mission

                    # destination_param = np.array([0,0,self.action[1],0,0,0])
                    destination_param = np.array([5,5,5,0,0,0])
                    
                    self.update_distance(destination_param)
                    trajectory_velocity = self.path.run(self)           # [  v0,  v1,  v2,  v3  ] 


                    velocity_list = Float32MultiArray()
                    velocity_list.data = trajectory_velocity  
                    self.pub2motion_motion.publish(self.action)     
                    self.pub2motion_trajec.publish(velocity_list)

                    self.action = None
                    

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass


if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()