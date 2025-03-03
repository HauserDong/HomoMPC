################################################################
# 
# Author: Mike Chen, Hauser Dong
# From Peking university
# Last update: 2024.03.03
# 
################################################################


import numpy as np
from .Omnidirection import omnidirection

class mini_mec(omnidirection):

    def __init__(self, index, radius, K, h, ini_p, target,obs_env_idx):

        # the shape of this agent
        shape=radius

        # maximum acc
        Umax=2.0

        # maximum velocity
        Vxmax=0.8

        Vymax=0.6

        buffer=0.2

        super().__init__(index, K, h, ini_p, target,shape=shape,Vxmax=Vxmax,Vymax=Vymax,buffer=buffer,Umax=Umax,obs_env_idx=obs_env_idx)
