#!/usr/bin/python3
import numpy as np
import rospy
import matplotlib.pyplot as plt
import scipy.sparse as ssp
import scipy.sparse.linalg as sla

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
        self.x_des = np.array([0,0,5,0,0,0])                                          # need input

        self.distance = np.sum((self.x_des-self.x_0)**2)**0.5
        self.T = round(self.distance*2)                                               
        if self.T <= 1:                                                          
            self.T = 1  
        self.n = self.T*10

        self.gen_number = 1                                                           # slice generation number


    def action_update(self,action_msgs):

        self.action = action_msgs.data

    def update_distance(self):
        self.distance = np.sum((self.x_des-self.x_0)**2)**0.5

    def update_state(self,data_hub):            ## need memory connection
        self.x_0 = np.array([data_hub.pos_n,data_hub.pos_e,data_hub.pos_d,data_hub.vel_n,data_hub.vel_e,data_hub.vel_d])



    def run(self):
        while not rospy.is_shutdown():
            if self.action != None:
                # if self.action != "park" and "search" and "take_off":
                    
                #     print(self.action)
                #     self.pub2motion_motion.publish(self.action) 
                #     # if the mission is simple, pass the mission to motion controller
                #     self.action = None

                if self.action == "take_off":
                    self.pub2motion_motion.publish(self.action)
                    # self.update_state()                               ## update x_0 (memory connection)
                    self.update_distance()                              ## update distance (simple calculation)
                    self.generate()                                     ## generate trajectory with x_0 (input : x_0 / output : velocity list)

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass
    
    def generate(self):             ## input : x_0 by state_update / output : velocity list ,4 front

        x_0 = self.x_0
        x_des = self.x_des
  

        delt = self.T/self.n
        gamma = 0.05

        A = np.zeros((6,6))
        B = np.zeros((6,3))

        A[0,0] = 1
        A[1,1] = 1
        A[2,2] = 1
        A[0,3] = (1-gamma*delt/2)*delt
        A[1,4] = (1-gamma*delt/2)*delt
        A[2,5] = (1-gamma*delt/2)*delt
        A[3,3] = 1-gamma*delt
        A[4,4] = 1-gamma*delt
        A[5,5] = 1-gamma*delt

        B[0,0] = delt**2/2
        B[1,1] = delt**2/2
        B[2,2] = delt**2/2
        B[3,0] = delt
        B[4,1] = delt
        B[5,2] = delt


        G = np.zeros((6,3*self.n))                                                              # Gu = x_des - (A^n)*(x_0)

        for i in range(self.n):                                                                 ## set G matrix
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,self.n-i-1))@B
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,self.n)@x_0)[0]                     # Optimize

        u_vec = u_hat                                                                           # save
        u_opt = u_vec.reshape(self.n,3).T                                                       # reshpae
        x = np.zeros((6,self.n+1))                                                              # saver
        x[:,0] = x_0                                                                            # set x_0
        for t in range(self.n):                                                                 # save [x,y,z,Vx,Vy,Vz]
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])
        x = x.T                                                                                 # x : [[x0],[x1],[x2]...]
                                                                                                # xn : [Pos_x,Pox_y,Pos_z,Vel_x,Vel_y,Vel_z]

        velocity_list = Float32MultiArray()                                                     # publish
        velocity_list.data = np.reshape(x[0:4,3:6],(1,12))[0]                                   ## cutting velocity 4 front
        self.pub2motion_trajec.publish(velocity_list)

        print('velo data number {}= '.format(self.gen_number), velocity_list.data.reshape((4,3))[:,2])

        self.x_0 = x[4,:]   # need change ( input / state update )

        if self.T <= 1:                                                                     # last trajectory
            self.T = 1
            self.action = None
        else :
            self.T -= 0.4
            self.n -= 4
            self.gen_number += 1

if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()