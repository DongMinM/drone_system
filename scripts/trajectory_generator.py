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
        self.x_0 = np.array([0,0,0,0,0,0])
        self.x_des = np.array([0,0,10,0,0,0])

    def action_update(self,action_msgs):

        self.action = action_msgs.data
        print(123)
        # self.x_0 = 


    def run(self):
        while not rospy.is_shutdown():
            if self.action != None:
                # if self.action != "park" and "search" and "take_off":
                    
                #     print(self.action)
                #     self.pub2motion_motion.publish(self.action) 
                #     # if the mission is simple, pass the mission to motion controller
                #     self.action = None
                if self.action == "take_off":
                    self.x_0 = np.array([0,0,5,0,0,0])
                    self.pub2motion_motion.publish(self.action) 
                    self.generate()

                elif self.action == "park":

                    pass

                elif self.action == "search":

                    pass
    
    def generate(self):
        distance = np.sum((self.x_des-self.x_0)**2)**0.5
        T = round(distance*2)
        if T <= 4:
            T = 4       
        n = T*10
        ts=np.linspace(0,T,n+1)
        delt = T/n
        gamma = 0.05


        A = np.zeros((6,6))
        B = np.zeros((6,3))
        C = np.zeros((3,6))

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

        x_0 = self.x_0
        x_des = self.x_des

        G = np.zeros((6,3*n))

        for i in range(n):
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,n-i-1))@B
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,n)@x_0)[0]

        u_vec = u_hat
        u_opt = u_vec.reshape(n,3).T
        x = np.zeros((6,n+1))
        x[:,0] = x_0
        for t in range(n):
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])
        x = x.T
        # print('velo : ', x[:,3:6])
        
        # if n == 4:
        #     self.arrive = 1
        # else:
        #     self.arrive = 0
        velocity_list = Float32MultiArray()
        print(np.reshape(x[0:4,3:6],(1,12)))
        velocity_list.data = np.reshape(x[0:4,3:6],(1,12))[0]
        print(velocity_list)
        self.pub2motion_trajec.publish(velocity_list)
        self.action = None
if __name__ == "__main__":

    T = TrajectoryGenerator()
    T.run()