################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.26
# 
################################################################


import numpy as np
from .Omnidirection import omnidirection

class mini_om(omnidirection):

    def __init__(self, index, radius, K, h, ini_p, target,obs_env_idx):

        # the shape of this agent
        shape=radius

        # maximum acc
        Umax=2.0

        # maximum velocity
        Vxmax=0.6

        Vymax=0.6

        buffer=0.2

        super().__init__(index, K, h, ini_p, target,shape=shape,Vxmax=Vxmax,Vymax=Vymax,buffer=buffer,Umax=Umax,obs_env_idx=obs_env_idx)
