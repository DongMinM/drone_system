import numpy as np
import scipy.sparse as ssp
import scipy.sparse.linalg as sla


class trajectoyu_velo_Gen:

    def __init__(self):
        pass

    def run(self,cur_status):             ## input : x_0 by state_update / output : velocity list ,4 front

        x_0 = cur_status.x_0
        x_des = cur_status.x_des
  

        delt = cur_status.Time/cur_status.n
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


        G = np.zeros((6,3*cur_status.n))                                                              # Gu = x_des - (A^n)*(x_0)

        for i in range(cur_status.n):                                                                 ## set G matrix
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,cur_status.n-i-1))@B
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,cur_status.n)@x_0)[0]                     # Optimize

        u_vec = u_hat                                                                           # save
        u_opt = u_vec.reshape(cur_status.n,3).T                                                       # reshpae
        x = np.zeros((6,cur_status.n+1))                                                        # saver
        x[:,0] = x_0                                                                            # set x_0
        for t in range(cur_status.n):                                                           # save [x,y,z,Vx,Vy,Vz]
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])
        x = x.T                                                                                 # x : [[x0],[x1],[x2]...]
                                                                                                # xn : [Pos_x,Pox_y,Pos_z,Vel_x,Vel_y,Vel_z]
        cur_status.x_0 = x[4,:]   # need change ( input / state update )


        if cur_status.Time <= 1:                                          # last trajectory
            cur_status.Time = 1
            cur_status.gen_number = 0                                     # finish and reset
            cur_status.action = None
        else :
            cur_status.Time -= 0.4
            cur_status.n -= 4
            cur_status.gen_number += 1
            # cur_status.action = None

        velocity_4_front = np.reshape(x[0:4,3:6],(1,12))[0]               # 4 front of velocity list
        
        print('velo data number {}= '.format(cur_status.gen_number),velocity_4_front.reshape((4,3))[:,2])


        return velocity_4_front
