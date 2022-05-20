# Applevision ROS package

## Setup

You can install this package and it's dependencies as follows:
0. Install ROS melodic on your machine.
1. Grab a copy of [`applevision.rosinstall`](./applevision.rosinstall).
2. Run the following to clone everything into your workspace using [wstool](http://wiki.ros.org/wstool), install dependencies, and build:
```sh
# in catkin_ws
catkin init
source devel/setup.bash
catkin build

wstool init src
wstool merge -t src applevision.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

cd /usr/lib/x86_64-linux-gnu
pip3 install --upgrade pip
pip3 install empy
sudo apt install ros-melodic-moveit
pip3 install numpy opencv-python-headless

catkin build
```

## Launching

### Simulated Robot

```
roslaunch applevision_moveit_config demo.launch
roslaunch applevision_rospkg fake_sensor.launch
src/applevision_rospkg/bin/applevision_motion.py
```

### Real Robot

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=169.254.177.232
roslaunch applevision_moveit_config ur5e_moveit_planning_execution.launch
roslaunch applevision_moveit_config moveit_rviz.launch
roslaunch applevision_rospkg real_sensor_robot.launch
src/applevision_rospkg/bin/applevision_motion.py
```