################################################################
# 
# Author: Hauser Dong
# From Peking university
# Last update: 2025.03.03
# 
###############################################################


import numpy as np
import rospy
from geometry import *

'''
test_mode:
    0: single path planning
    1: only passage segment detection
    2: no replanning
    3: pure replanning
    4: homotopy-aware optimal path planning
'''
test_mode = rospy.get_param('test_mode', default=4)

replanning = True   # for test_mode = 0

rgb_color_list = [
    [237/255, 27/255, 52/255],      # red
    [78/255, 136/255, 199/255],     # blue
    [77/255, 184/255, 72/255],      # green
    [252/255, 179/255, 21/255],     # warm yellow
    [80/255, 196/255, 221/255],     # Aquamarine
    [187/255, 137/255, 202/255],    # Lavender
    [203/255, 219/255, 42/255],     # Lime green
    [236/255, 143/255, 156/255],    # Salmon
]

'''
'index': the index of agent (start from 0)
'K': horizon length
'h': sampling time interval
't_w': waiting time (set as constant in this work)
't_c': computation time (set as constant in this work)
'type': agent type, for type can be found at 'src/planner/scripts/Dynamic' (not used in this work)
'state': initial state
'tar': target state
'radius': radius of the agent
'com_radius': communication radius (set as a huge constant in this work)
'''
t_w = 0.20
t_c = 0.10
com_r = 20.0
radius = 0.2

global agent_list
agent_list = [

    {
        'index': 0,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([0.5, 1.0]),
        'tar': np.array([4.5, 4.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 1,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([0.5, 2.0]),
        'tar': np.array([4.5, 3.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 2,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([0.5, 3.0]),
        'tar': np.array([4.5, 2.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 3,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([0.5, 4.0]),
        'tar': np.array([4.5, 1.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 4,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([4.5, 4.0]),
        'tar': np.array([0.5, 1.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 5,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([4.5, 3.0]),
        'tar': np.array([0.5, 2.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 6,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([4.5, 2.0]),
        'tar': np.array([0.5, 3.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },

    {
        'index': 7,
        'K': 12,
        'h': 0.2,
        't_w': t_w,
        't_c': t_c,
        'type':'Mini_mec',
        'state': np.array([4.5, 1.0]),
        'tar': np.array([0.5, 4.0]),
        'radius': np.array([radius]),
        'com_radius': com_r,
    },
]


# Obstacle Construction

global map_range
map_range={'x':[0.0, 5.0],'y':[0.0, 5.0]}     # range of the map

# discretization resolution
global resolution
resolution=0.1

ini_obstacle_list=[
    [   
        polygon([np.array([1.5, 0.8]),np.array([1.8, 0.8]),np.array([1.8, 1.2]), np.array([1.5, 1.2])]),
        polygon([np.array([1.5, 1.8]),np.array([1.8, 1.8]),np.array([1.8, 2.2]), np.array([1.5, 2.2])]),
        polygon([np.array([1.5, 2.8]),np.array([1.8, 2.8]),np.array([1.8, 3.2]), np.array([1.5, 3.2])]),
        polygon([np.array([1.5, 3.8]),np.array([1.8, 3.8]),np.array([1.8, 4.2]), np.array([1.5, 4.2])]),
        polygon([np.array([2.5,0.3]),np.array([2.8,0.3]),np.array([2.8, 0.7]), np.array([2.5, 0.7])]),
        polygon([np.array([2.5,1.3]),np.array([2.8,1.3]),np.array([2.8, 1.7]), np.array([2.5, 1.7])]),
        polygon([np.array([2.5,2.3]),np.array([2.8,2.3]),np.array([2.8, 2.7]), np.array([2.5, 2.7])]),
        polygon([np.array([2.5,3.3]),np.array([2.8,3.3]),np.array([2.8, 3.7]), np.array([2.5, 3.7])]),
        polygon([np.array([2.5,4.3]),np.array([2.8,4.3]),np.array([2.8, 4.7]), np.array([2.5, 4.7])]),
        polygon([np.array([3.5, 0.8]),np.array([3.8, 0.8]),np.array([3.8, 1.2]), np.array([3.5, 1.2])]),
        polygon([np.array([3.5, 1.8]),np.array([3.8, 1.8]),np.array([3.8, 2.2]), np.array([3.5, 2.2])]),
        polygon([np.array([3.5, 2.8]),np.array([3.8, 2.8]),np.array([3.8, 3.2]), np.array([3.5, 3.2])]),
        polygon([np.array([3.5, 3.8]),np.array([3.8, 3.8]),np.array([3.8, 4.2]), np.array([3.5, 4.2])]),
    ] 
]