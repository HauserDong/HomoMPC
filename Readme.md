# Homotopy-aware Multi-agent Navigation via Distributed Model Predictive Control

This repository provides an algorithm implementation of the paper ***Homo**topy-aware Multi-agent Navigation via Distributed **M**odel **P**redictive **C**ontrol*. 
The details of this work are as follows:
* **Authors**: Haoze Dong, Meng Guo, Chengyi He and Zhongkui Li
* **Paper**: This paper has been accepted by [IROS 2025](https://www.iros25.org). See the full length version: [arXiv](https://arxiv.org/abs/2507.19860).
* **Video**: [YouTube](https://youtu.be/9HCiO9QTpgw)

## Dependencies Installation

The system is tested on `Ubuntu 20.04` with `Python 3`.

**Install Required Python Packages**

```
pip install numpy scipy pybind11==2.11.1 rospkg==1.5.0 setuptools==70.0.0
```

**Install Eigen**
```
sudo apt-get install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

**Install ROS**

Follow the official ROS installation guide: [ROS Installation](https://www.ros.org/)

**Install Acados**

Refer to the Acados documentation to install Acados and its Python interface:
[Acados Python interface](https://docs.acados.org/python_interface/index.html)

>**Note**: t_renderer binaries may need to be built manually from [tera_renderer](https://github.com/acados/tera_renderer/releases)

**Install pcl for obstacle visualization**
```
sudo apt install python3-pcl
sudo apt install ros-noetic-pcl-ros
```

**Install tmux**
```
sudo apt-get install tmux
```


## Build Instruction

**Build up the workspace**
```
git clone https://github.com/HauserDong/HomoMPC
cd HomoMPC
catkin_make
source devel/setup.bash
```

**Build GJK & Homotopy-aware Path Planning algorithm code**
>**Note**: Before running the command below, ensure you are in the root of your workspace.

```
source easy_build.sh
```

## Running the simulation

>**Important**: Always source the workspace `devel/setup.bash` script when opening a new terminal. *Hint*: you can add it to your `.bashrc` for convenience.

**Launch the System**
```
roslaunch bring_up bring_up.launch 
```

**Run the planner**
```
python3 src/local_planner/planner/scripts/launch.py
```
> To terminate the tmux windows, try `tmux kill-server` in your command line.

**Publish targets**
```
python3 src/local_planner/planner/scripts/publish.py
```

## Acknowledgements
The implementation of this project is founded upon the following packages.

IMPC-OB: https://github.com/PKU-MACDLab/IMPC-OB

ASAP: https://github.com/CYDXYYJ/ASAP

acados: https://github.com/acados/acados

HPSP: https://github.com/HuangJingGitHub/HPSP

tuw_multi_robot: https://github.com/tuw-robotics/tuw_multi_robot

openGJK: https://www.mattiamontanari.com/opengjk/
