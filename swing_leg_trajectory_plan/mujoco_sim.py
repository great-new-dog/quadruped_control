import mujoco as mj
from mujoco.glfw import glfw
import mujoco.viewer
import numpy as np
import time


class MujocoSim:
    def __init__(self, model_path):
        self.model = mj.MjModel.from_xml_path(model_path)
        self.data = mj.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.render = mj.Renderer(self.model)
        self.sim_dt = self.model.opt.timestep
        mj.mj_resetDataKeyframe(self.model, self.data, 0)
        
        
    def controller(self, qpos, qvel, qpos_des, qvel_des, kp, kd):
        self.data.ctrl = kp * (qpos_des - qpos) + kd * (qvel_des - qvel)
    
    def getSensorData(self):
        data = self.data.sensordata
        qpos = data[0:12]
        qvel = data[12:24]
        return qpos, qvel
    
    def getImage(self):
        self.render.update_scene(self.data)
        image = self.render.render()
        return image
    
    def step(self):
        mj.mj_step(self.model, self.data)
        self.viewer.sync()
        time.sleep(self.sim_dt)