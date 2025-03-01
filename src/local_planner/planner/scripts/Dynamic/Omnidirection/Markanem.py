################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.3.17
# 
################################################################


import numpy as np
from .Omnidirection import omnidirection

class markanem(omnidirection):

    def __init__(self, index, radius, K, h, ini_p, target, obs_env_idx):

        # the shape of this agent
        shape=radius

        # maximum acc
        Umax=1.5

        # maximum velocity
        Vxmax=1.0

        Vymax=1.0


        buffer=0.1

        super().__init__(index, K, h, ini_p, target,shape=shape,Vxmax=Vxmax,Vymax=Vymax,buffer=buffer,Umax=Umax,obs_env_idx=obs_env_idx)
