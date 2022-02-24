import numpy as np
import scipy.sparse as ssp
import scipy.sparse.linalg as sla


class Trajectory_Velocity_Generator:
    def __init__(self):
        pass

    def run(self,trajectory_data):             ## input : x_0 by state_update / output : velocity list ,4 front

        x_0 = trajectory_data.x_0
        x_des = trajectory_data.x_des
  

        delt = trajectory_data.Time/trajectory_data.n
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


        G = np.zeros((6,3*trajectory_data.n))                                                              # Gu = x_des - (A^n)*(x_0)

        for i in range(trajectory_data.n):                                                                 ## set G matrix
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,trajectory_data.n-i-1))@B
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,trajectory_data.n)@x_0)[0]                     # Optimize

        u_vec = u_hat                                                                           # save
        u_opt = u_vec.reshape(trajectory_data.n,3).T                                                       # reshpae
        x = np.zeros((6,trajectory_data.n+1))                                                        # saver
        x[:,0] = x_0                                                                            # set x_0
        for t in range(trajectory_data.n):                                                           # save [x,y,z,Vx,Vy,Vz]
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])
        pos_and_velocity = x.T                                                                                 # x : [[x0],[x1],[x2]...]
                                                                                         # xn : [Pos_x,Pox_y,Pos_z,Vel_x,Vel_y,Vel_z]
    
        trajectory_data.x_0 = pos_and_velocity[4,:]   # need change ( input / state update )


        if trajectory_data.Time <= 0.4:                                          # last trajectory
            trajectory_data.Time = 0.4
            trajectory_data.gen_number = 0                                     # finish and reset
            trajectory_data.action = None

        else :
            trajectory_data.Time -= 0.4
            trajectory_data.n -= 4
            trajectory_data.gen_number += 1
            # trajectory_data.action = None

        velocity_4_front = np.reshape(pos_and_velocity[0:4,0:3],(1,12))[0]               # 4 front of velocity list
        # print(np.append(np.array([trajectory_data.gen_number]),velocity_4_front,axis=0))

        return np.append(np.array([trajectory_data.gen_number]),velocity_4_front,axis=0)
