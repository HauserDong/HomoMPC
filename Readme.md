# Homotopy-aware Multi-agent Navigation via Distributed Model Predictive Control

This repository provides an algorithm implementation of the paper *Homotopy-aware Multi-agent Navigation via Distributed Model Predictive Control*. 
The details of this work are as follows:
* **Authors**: Haoze Dong, Meng Guo, Chengyi He and Zhongkui Li
* **Paper**: to be released
* **Video**: to be released

## Dependencies Installation

The system is tested on `Ubuntu 20.04` with `Python 3.8.10`.

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

## Build Instruction

>**Note**: Before running any commands below, ensure you are in the root of your workspace.

**Build up the workspace**
```
catkin_make
```

**Build GJK & Homotopy-aware Path Planning algorithm code**

```
source easy_build.sh
```

## Running the simulation

>**Important**: Always source the workspace `devel/setup.bash` script when opening a new terminal. *Hint*: you can add it to your .bashrc for convenience.

**Launch the System**
```
roslaunch bring_up bring_up.launch 
```

**Run the planner**
```
python3 src/local_planner/planner/scripts/launch.py
```

**Publish targets**
```
python3 src/local_planner/planner/scripts/publish.py
```