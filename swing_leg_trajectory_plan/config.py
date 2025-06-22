import numpy as np
class Config:
    hip_len = 0.08
    thigh_len = 0.213
    shank_len = 0.213
    body_len_x= 0.1881
    body_len_y = 0.04675
    
    body_mass = 5.204
    body_inertia = np.array([0.0168101, 0, 0,
                             0, 0.0630105, 0,
                             0, 0, 0.0716565])
    body_height = 0.26