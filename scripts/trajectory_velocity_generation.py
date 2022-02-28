from asyncio import gather
import numpy as np
import scipy.sparse as ssp
import scipy.sparse.linalg as sla


class Trajectory_Velocity_Generator:
    def __init__(self):
        pass

    def run(self,trajectory_data):             

        x_0 = trajectory_data.current_status              ## simple resave
        x_des = trajectory_data.x_des          ## simple resave

        """ matrices

                X(t+1) = A@x(t) + B@u(t)          ==>   none reflect gravity
                        or
                X(t+1) = A@x(t) + B@u(t) + B@g    ==>   reflect gravity

                Gu = x_des - (A^n)*(x_0)          ==>   total matrix , none reflect gravity
                u : total acceleration

                Gu = x_des - (A^n)*(x_0) - G@g    ==>   total matrix , reflect gravity
                u : acceleration of motor force
                
                ==> same total acceleration

                below is just matrices setup
        """
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

        g = np.array([0,0,-9.8]*trajectory_data.n)
        G = np.zeros((6,3*trajectory_data.n))

        for i in range(trajectory_data.n):                                                                
            G[:,3*i:3*(i+1)] = np.linalg.matrix_power(A,max(0,trajectory_data.n-i-1))@B

        """ The top is just matrices setup """
                                                              
        """ Optimization """

        # u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,trajectory_data.n)@x_0 - G@g.T )[0]          # reflect gravity
        u_hat = sla.lsqr(G,x_des - np.linalg.matrix_power(A,trajectory_data.n)@x_0 )[0]                    # no reflect gravity

        u_vec = u_hat                                                                                      # just save
        u_opt = u_vec.reshape(trajectory_data.n,3).T                                                       # just reshape
        g_re = g.reshape(trajectory_data.n,3).T                                                            # just reshape
        x = np.zeros((6,trajectory_data.n+1))                                                              # just declaration
        x[:,0] = x_0                                                                                       # just set x_0

        for t in range(trajectory_data.n):                                                                 # calculation pos and velocity
            # x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t]) + B.dot(g_re[:,t])                              # X(t+1) = A@x(t) + B@u(t) - G@g
            x[:,t+1] = A.dot(x[:,t]) + B.dot(u_opt[:,t])                                                   # X(t+1) = A@x(t) + B@u(t)

        pos_and_velocity = x.T                                                                             # [[x0],[x1],[x2]...]
                                                                                                           # x : [Pos_x,Pox_y,Pos_z,Vel_x,Vel_y,Vel_z]
    
        trajectory_data.x_0 = pos_and_velocity[4,:]   # need change ( input / state update )

        """ if drone is near the destination, generate last trajectory """
        if trajectory_data.Time -0.4 <= 0.4:                                          # next trajectory time < 0.4 ==> near the destination
            print("last")
            trajectory_data.Time = 0.4                                                # set last time
            trajectory_data.n = 4                                                     # set last way point number
            trajectory_data.path_number = -1                                           # finish triger
            trajectory_data.action = None

        else :
            trajectory_data.Time -= 0.4                                               # set next trajectory time
            trajectory_data.n -= 4                                                    # set next trajectory way point number
            trajectory_data.path_number += 1                                           # trajectory generation number
            # trajectory_data.action = None

        velocity_4_front = np.reshape(pos_and_velocity[0:4,3:6],(1,12))[0]                  # 4 front of velocity list
        print(np.append(np.array([trajectory_data.path_number]),velocity_4_front,axis=0))

        return np.append(np.array([trajectory_data.path_number]),velocity_4_front,axis=0)
