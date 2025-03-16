import numpy as np 
import math
from forword_kinematics import fk
from config import *

def jocobian(theta_1, theta_2, theta_3, leg):
    if leg%2==0:
        epslon = -1
    else:
        epslon = 1
    
    l1 = Config.hip_len
    l2 = Config.thigh_len
    l3 = Config.shank_len
    
    j11 = 0
    j12 = -l2*math.cos(theta_2)-l3*math.cos(theta_2+theta_3)
    j13 = -l3*math.cos(theta_2+theta_3)
    j21 = -epslon*l1*math.sin(theta_1)+l2*math.cos(theta_1)*math.cos(theta_2) + l3*math.cos(theta_1)*math.cos(theta_2+theta_3)
    j22 = -l2*math.sin(theta_1)*math.sin(theta_2)-l3*math.sin(theta_1)*math.sin(theta_2+theta_3)
    j23 = -l3*math.sin(theta_1)*math.sin(theta_2+theta_3)
    j31 = epslon*l1*math.cos(theta_1)+l2*math.sin(theta_1)*math.cos(theta_2) + l3*math.sin(theta_1)*math.cos(theta_2+theta_3)
    j32 = l2*math.cos(theta_1)*math.sin(theta_2)+l3*math.cos(theta_1)*math.sin(theta_2+theta_3)
    j33 = l3*math.cos(theta_1)*math.sin(theta_2+theta_3)
    
    return np.array(
        [[j11, j12, j13],
         [j21, j22, j23],
         [j31, j32, j33]
        ]
    )
    
def ik_num(px, py, pz, leg):
    theta = np.array([0.01, -0.01, 1.5]).transpose()
    eps = 0.00001
    max_iter = 10000
    it = 0
    while it < max_iter:
        theta_1 = theta[0]
        theta_2 = theta[1]
        theta_3 = theta[2]
        
        px_t, py_t, pz_t = fk(theta_1, theta_2, theta_3, leg)
        if it == 490:
            print(f'x: {px_t}, y: {py_t}, z: {pz_t}')
        err = np.array([px-px_t, py-py_t, pz-pz_t]).transpose()
        err_norm = math.sqrt((px_t-px)**2 + (py_t-py)**2 + (pz_t-pz)**2)
        if err_norm < eps:
            break
        J = jocobian(theta_1, theta_2, theta_3, leg)
        
        delta_theta = np.linalg.inv(J) @ err 
        theta = theta + delta_theta * 0.025
        it = it + 1
    print(it)
    print(fk(theta_1, theta_2, theta_3, leg))
    return theta


def main():
    px = -0.1*math.sqrt(3)
    py = -0.1
    pz = -0.3
    leg = 0
    theta = ik_num(px, py, pz, leg)
    print(f'theta_1: {theta[0]*180/math.pi}, theta_2: {theta[1]*180/math.pi}, theta_3: {theta[2]*180/math.pi}')

if __name__ == '__main__':
    main()