# Applevision ROS package

## Setup

Setup UR5 drivers (also outlined in [this google doc](https://docs.google.com/document/d/1HJujs0zgqWxUMx5Ktua1PtX73_GnM81Z3KrTZ6-BJZs/edit#)):
0. Install ROS melodic on Ubuntu 18.04.
1. Follow the instructions on the [UR5 github repository](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#building) to install the UR5 drivers and moveit.
2. Clone this repository into catkin_ws/src and install its dependencies using `rosdep install --from-paths src --ignore-src -y`.
3. Use the launch files to get started:
    * `roslaunch applevision_rospkg fake_sensor.launch` for fake simulated robot
    * `roslaunch applevision_rospkg read_sensor_robot.launch` for real robot (UNTESTED)
