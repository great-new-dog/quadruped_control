from matplotlib import animation
from mujoco_sim import MujocoSim
from fk import fk
from ik import ik
from interpolation import getCubicSpline, getLegTrajectory
import os
import time
import matplotlib.pyplot as plt
import imageio

def main():
    pth = os.path.dirname(os.path.abspath(__file__))
    pth_full = os.path.join(pth, 'unitree_go1', 'scene.xml')

    sim = MujocoSim(pth_full)
    # which leg to control, 0-3
    leg = 0
    # init qpos
    init_theta = [0.0, 0.8, -1.6, 0.0, 0.8, -1.6,0.0, 0.8, -1.6,0.0, 0.8, -1.6]
    
    sim_dt = 0.002
    sim_cnt = 0
    sim_start = 0.1
    period = 0.5
    sim_end = sim_start + period + sim_dt + 0.1

    runTime = 0
    first = True

    kp = 20
    kd = 5
    
    # set target
    qpos_leg = init_theta[leg*3:leg*3+3].copy()
    px_init, py_init, pz_init = fk(qpos_leg[0], qpos_leg[1], qpos_leg[2], leg)
    print(f'Initial foot position: px: {px_init}, py: {py_init}, pz: {pz_init}')
    px_final = px_init + 0.25
    py_final = py_init
    pz_final = pz_init
    height = pz_init + 0.2
    print(f'Final foot position: px: {px_final}, py: {py_final}, pz: {pz_final}')
    
    # control variables
    qpos_cmd = init_theta.copy()
    qvel_cmd = [0.0, 0.0, 0.0] * 4
    
    # record variables
    runTimeList = []
    qposList = []
    qvelList = []
    pfootList = []
    qposcmdList = []
    qvelcmdList = []
    imageList = []
    
    while runTime < sim_end:
        sim_cnt += 1
        runTime = sim_cnt * sim_dt
        
        # get sensor data
        qpos, qvel = sim.getSensorData()
        
        if runTime < sim_start:
            pass
        elif runTime < sim_start + period + sim_dt:
            # record the data
            runTimeList.append(runTime - sim_start)
            qpos_leg = qpos[leg*3:leg*3+3].copy()
            qvel_leg = qvel[leg*3:leg*3+3].copy()
            qposList.append(qpos_leg)
            qvelList.append(qvel_leg)
            px, py, pz = fk(qpos_leg[0], qpos_leg[1], qpos_leg[2], leg)
            pfootList.append([px, py, pz])
            
            # calculate the cmd
            pfoot_cmd = getLegTrajectory([px_init, py_init, pz_init], [0.0, 0.0, 0.0], [px_final, py_final, pz_final], [0.0, 0.0, 0.0], runTime - sim_start, period, height)
            pfoot_next_cmd = getLegTrajectory([px_init, py_init, pz_init], [0.0, 0.0, 0.0], [px_final, py_final, pz_final], [0.0, 0.0, 0.0], runTime - sim_start + sim_dt, period, height)            
            theta_1, theta_2, theta_3 = ik(pfoot_cmd[0], pfoot_cmd[1], pfoot_cmd[2], leg)
            theta_1_next, theta_2_next, theta_3_next = ik(pfoot_next_cmd[0], pfoot_next_cmd[1], pfoot_next_cmd[2], leg)
            vel_cmd = [(theta_1_next - theta_1)/sim_dt, (theta_2_next - theta_2)/ sim_dt, (theta_3_next - theta_3)/ sim_dt] 
            
            qpos_cmd[leg*3:leg*3+3] = [theta_1, theta_2, theta_3]
            qvel_cmd[leg*3:leg*3+3] = vel_cmd[:]
            qposcmdList.append(qpos_cmd[leg*3:leg*3+3].copy())
            qvelcmdList.append(qvel_cmd[leg*3:leg*3+3].copy())
            
        else:
            if first is True:
                first = False
                qpos_cmd = qpos.copy()
                qvel_cmd = [0.0, 0.0, 0.0] * 4

        # sim step
        sim.controller(qpos, qvel, qpos_cmd, qvel_cmd, kp, kd)
        sim.step()
        
        image = sim.getImage()
        imageList.append(image)
    sim.viewer.close()
    
    # plot the results
    """
        plot leg trajectory
    """
    fig, axis = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('Leg Trajectory')
    titleName = ['x', 'y', 'z']
    for i in range(3):
        axis[i].plot(runTimeList, [p[i] for p in pfootList])
        axis[i].set_title(titleName[i])
        axis[i].grid()
    axis[-1].set_xlabel('Time (s)')
    
    """
        plot joint angle
    """
    fig, axis = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('Joint Angle')
    titleName = ['ABAD', 'HIP', 'KNEE']
    for i in range(3):
        axis[i].plot(runTimeList, [qposcmd[i] for qposcmd in qposList], label='cmd')
        axis[i].plot(runTimeList, [qpos[i] for qpos in qposList], label='actual')
        axis[i].set_title(titleName[i])
        axis[i].legend()
        axis[i].grid()
    axis[-1].set_xlabel('Time (s)')
    
    """
        plot joint velocity
    """
    fig, axis = plt.subplots(nrows=3, ncols=1, sharex=True)
    fig.suptitle('Joint Angle')
    titleName = ['ABAD', 'HIP', 'KNEE']
    for i in range(3):
        axis[i].plot(runTimeList, [qvelcmd[i] for qvelcmd in qvelcmdList], label='cmd')
        axis[i].plot(runTimeList, [qvel[i] for qvel in qvelList], label='actual')
        axis[i].set_title(titleName[i])
        axis[i].legend()
        axis[i].grid()
    axis[-1].set_xlabel('Time (s)')
    plt.show()
    
    # imageio.mimsave('ani.gif', imageList[::10], fps=60)
    
if __name__ == "__main__":
    main()