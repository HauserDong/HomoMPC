Tested on Ubuntu 20.04, python 3

Dependencies installation:
1. pip install numpy scipy pybind11 rospkg==1.4.0 setuptools==70.0.0

Install Eigen:
1. sudo apt-get install libeigen3-dev
2. sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

Install ROS:
1. Install ROS following https://www.ros.org/

Install Acados:
1. Install ACADOS and its Python interface (https://docs.acados.org/python_interface/index.html)
(NOTE: t_renderer binaries may be built by your own from https://github.com/acados/tera_renderer/releases)

Install pcl for obstacle visualization (not used on board):
1. sudo apt install python3-pcl
2. sudo apt install ros-noetic-pcl-ros

###############################################################
## By default, please enter the root of your workspace first ##
###############################################################

Build up the workspace:
1. catkin_make

GJK & Hybrid A* Preparation:
1. source easy_build.sh

Generate grid map for each agent (if you use hybrid A* for path planning):
1. python3 src/planner/scripts/generate_grid_map.py

Run simulation:
1. roslaunch bring_up bring_up.launch 
2. python3 src/local_planner/planner/scripts/launch.py         (no number after)
3. python3 src/local_planner/planner/scripts/publish.py

Run multiple simulation:
1. roslaunch bring_up start_mul.launch 
2. python3 src/local_planner/planner/scripts/run_mul.py
注：testcase 3, 4, 13, 14 都有记录到过 "Timeout. Mission Failed"