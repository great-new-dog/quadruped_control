import numpy as np 
import math
from config import *

def ik(px, py, pz, leg):
    l1 = Config.hip_len
    l2 = Config.thigh_len
    l3 = Config.shank_len
    if leg%2==0:
        epslon = -1
    else:
        epslon = 1
    py = epslon * py
    
    L = math.sqrt(py**2 + pz**2 - l1**2)
    theta_1 = math.atan2(pz*l1+py*L, py*l1-pz*L)
    
    L = math.sqrt(px**2 + py**2 + pz**2 - l1**2)
    theta_3 = math.pi - math.acos((l2**2+l3**2-L**2)/(2*l2*l3))
    theta_3 = -theta_3
    
    a1 = py*math.sin(theta_1) - pz * math.cos(theta_1)
    a2 = px
    m1 = -l3*math.sin(theta_3)
    m2 = -l3*math.cos(theta_3)-l2
    theta_2 = math.atan2(a1*m1+a2*m2, a2*m1-a1*m2)
    
    return theta_1, theta_2, theta_3


def main():
    px = -0.1*math.sqrt(3)
    py = -0.1
    pz = -0.3
    leg = 0
    theta_1, theta_2, theta_3 = ik(px, py, pz, leg)
    print(f'theta_1: {theta_1}, theta_2: {theta_2}, theta_3: {theta_3}')

if __name__ == '__main__':
    main()