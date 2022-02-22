from scipy.optimize import minimize
import numpy as np
import cvxpy as cvx

def objective(a):
    return np.sum(np.square(a))


def constraint1(a):
    a = np.reshape(a,(4,3))
    # print(a)
    a0 = np.array(a[0])
    a1 = np.array(a[1])
    a2 = np.array(a[2])
    a3 = np.array(a[3])

    T = np.diag([0.1,0.1,0.1])
    A1 = 3*a0+2*a1+a2
    A2 = a0+a1+a2+a3
    H = (T**2)@A1.T+(T**2/2)@A2.T-np.array([0,0,1])

    return H

a0 = np.array([[0,0,1],[0,0,1],[0,0,1],[0,0,1]])

con2 = {'type':'eq','fun':constraint1}
cons = [ con2 ]
sol = minimize(objective,a0,method='SLSQP',constraints=cons)
sol_a = np.array(sol.x)
sol_a = np.reshape(sol_a,(4,3))
sol_a = np.round(sol_a,4)

print(sol_a)