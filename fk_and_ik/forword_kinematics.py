import numpy as np 
import math 
from config import *

def rotX(theta):
    return np.array(
        [[1, 0, 0],
         [0, math.cos(theta), -math.sin(theta)],
         [0, math.sin(theta), math.cos(theta)]            
        ]
    )

def rotY(theta):
    return np.array(
        [[math.cos(theta), 0, math.sin(theta)],
         [0, 1, 0],
         [-math.sin(theta), 0, math.cos(theta)]  
        ]
    )

def rotZ(theta):
    return np.array(
        [[math.cos(theta), -math.sin(theta), 0],
         [math.sin(theta), math.cos(theta), 0],
         [0, 0, 1]
        ]
    )

def fk(theta_1, theta_2, theta_3, leg):
    if leg%2 == 0:  # right
        epslon = -1
    else:           # left
        epslon = 1
        
    if leg <= 1:    # front
        delta = 1
    else:           # rear
        delta = -1
    
    pf_tmp1 = np.array([0, 0, -Config.thigh_len]).transpose() + \
        rotY(theta_3) @ np.array([0, 0, -Config.shank_len]).transpose()
    
    pf_tmp2 = np.array([0, epslon*Config.hip_len, 0]).transpose() + \
        rotY(theta_2) @ pf_tmp1
        
    pf = rotX(theta_1) @ pf_tmp2
    
    return pf

def jocobian(theta_1, theta_2, theta_3, leg):
    if leg%2==0:
        epslon = -1
    else:
        epslon = 1
    
    l1 = epslon * Config.hip_len
    l2 = -Config.thigh_len
    l3 = -Config.shank_len
    
    j11 = 0
    j12 = l2*math.cos(theta_2)+l3*math.cos(theta_2+theta_3)
    j13 = l3*math.cos(theta_2+theta_3)
    j21 = -l1*math.sin(theta_1)-l2*math.cos(theta_1)*math.cos(theta_2) - l3*math.cos(theta_1)*math.cos(theta_2+theta_3)
    j22 = l2*math.sin(theta_1)*math.sin(theta_2)+l3*math.sin(theta_1)*math.sin(theta_2+theta_3)
    j23 = l3*math.sin(theta_1)*math.sin(theta_2+theta_3)
    j31 = l1*math.cos(theta_1)-l2*math.sin(theta_1)*math.cos(theta_2) - l3*math.sin(theta_1)*math.cos(theta_2+theta_3)
    j32 = -l2*math.cos(theta_1)*math.sin(theta_2)-l3*math.cos(theta_1)*math.sin(theta_2+theta_3)
    j33 = -l3*math.cos(theta_1)*math.sin(theta_2+theta_3)
    
    return np.array(
        [[j11, j12, j13],
         [j21, j22, j23],
         [j31, j32, j33]
        ]
    )
    
def main():
    theta_1 = 0.0
    theta_2 = 0.0
    theta_3 = math.pi / 3
    leg = 0
    
    px, py, pz = fk(theta_1, theta_2, theta_3, leg)
    print(f'px: {px}, py: {py}, pz: {pz}')

if __name__ == '__main__':
    main()