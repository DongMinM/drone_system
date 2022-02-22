from scipy.optimize import minimize
import numpy as np
class Trajectory_generator:

    def __init__(self,v0,x0,xf):
        self.v0 = v0
        self.x0 = x0
        self.xf = xf

    def objective(self,a):                                          ## minimize a^2
        return np.sum(np.square(a))


    def constraint1(self,a):
        a = np.reshape(a,(4,3))
        # print(a)
        a1 = np.array(a[0])                                         ## 1st acceleration
        a2 = np.array(a[1])                                         ## 2nd acceleration
        a3 = np.array(a[2])                                         ## 3rd acceleration
        a4 = np.array(a[3])                                         ## ~

        T = np.diag([0.1,0.1,0.1])                                  ## time matrix
        A1 = 3*a1+2*a2+a3
        A2 = a1+a2+a3+a4
        H = (T**2)@A1.T+(T**2/2)@A2.T+4*T@self.v0.T-(self.xf-self.x0)

        return H

    def run(self):
        first_a = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1]])               ## first a pos in minimization

        con2 = {'type':'eq','fun':self.constraint1}
        cons = [ con2 ]                                                     ## set constraint

        sol = minimize(self.objective,first_a,method='SLSQP',constraints=cons)
        
        sol_a = np.array(sol.x).reshape((4,3)).round(4)                     ## shaping
        print(sol_a)                                                        ## m/s^2

if __name__ == '__main__':
    v0 = np.array([0,0,0])                                  ## initial velocity (m/s)
    x0 = np.array([0,0,0])                                  ## initial pos (m)
    xf = np.array([0,0,1])                                  ## final pos (m)
    trajectory_generator = Trajectory_generator(v0,x0,xf)
    trajectory_generator.run()