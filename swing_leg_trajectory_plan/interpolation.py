import numpy as np



def getCubicSpline(p0, v0, pf, vf, t, T):
    """
    Get cubic spline trajectory
    p0: initial pos
    v0: initial vel
    pf: final pos
    vf: final vel
    t: current time
    T: total time
    
    return: pt, the pos at time t
    """    
    
    if t <= 0:
        return p0
    elif t >= T:
        return pf
    
    tt = t / T
    a0 = p0
    a1 = v0 * T
    a2 = 3*(pf-p0) - (2*v0+vf)*T
    a3 = (v0+vf)*T - 2*(pf-p0)
    
    return a0 + a1*tt + a2*tt**2 + a3*tt**3

def getLegTrajectory(p0, v0, pf, vf, t, T, height):
    """
    Get leg trajectory
    p0: initial pos
    v0: initial vel
    pf: final pos
    vf: final vel
    t: current time
    T: total time
    
    return: pt, the leg pos at time t
    """
    if t <= 0:
        return p0
    elif t >= T:
        return pf
    
    px0, py0, pz0 = p0
    pxf, pyf, pzf = pf
    vx0, vy0, vz0 = v0
    vxf, vyf, vzf = vf
    px = getCubicSpline(px0, vx0, pxf, vxf, t, T)
    py = getCubicSpline(py0, vy0, pyf, vyf, t, T)
    
    if t < T / 2:
        pz = getCubicSpline(pz0, vz0, height, 0, t, T / 2)
    else:
        pz = getCubicSpline(height, 0, pzf, vzf, t - T / 2, T / 2)
        
    return [px, py, pz]

