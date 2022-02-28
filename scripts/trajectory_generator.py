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
        self.destination_status =  None                                        

        self.distance = 0
        self.Time = 0
        self.n = 0

        self.path_number = 0   

    def action_update(self,action_msgs):

        self.action = action_msgs.data


    def current_status(self,status):

        self.current_status = np.array([status.data.pos_n,
                                        status.data.pos_e,
                                        status.data.pos_d,
                                        status.data.vel_n,
                                        status.data.vel_e,
                                        status.data.vel_d])

    def destination_status(self,destination_params):

        ## calculate distance ( current status ~ destination )
        self.destination_status = destination_params
        self.distance = np.sum((self.destination_status[0:3]-self.current_status[0:3])**2)**0.5

        ## Only when first status is given, calculate time
        if self.path_number == 0:

                self.Time = round(self.distance*2)      # set average velocity
                self.n = self.Time*10
                
                ## time limit
                if self.Time <= 0.4:                
                    self.Time = 0.4  
                    self.n = self.Time*10


    def run(self):


        while not rospy.is_shutdown():

            if self.action != None:

                # simple mission
                if self.action[0] in [0,1,3]:
                    
                    print(self.action)
                    self.pub2motion_motion.publish(self.action) 
                    # if the mission is simple, pass the mission to motion controller
                    self.action = None

                # take_off mission
                elif self.action[0] == 2:
                    
                    print('take off',self.action[1:])
                    destination_params = np.array([0,0,self.action[1],0,0,0])
                    self.destination_status(destination_params)
                    
                    path_data = self.path.run(self)           # [ path number,  v0,  v1,  v2,  v3  ] 

                    velocity_list = Float32MultiArray()
                    velocity_list.data = path_data
                    self.pub2motion_motion.publish(velocity_list)     

                    self.action = None
                    

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass
                
                ## Reset path number when last path is published
                if self.path_number == -1:
                    self.path_number == 0

if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()