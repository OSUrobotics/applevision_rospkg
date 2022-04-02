# Applevision ROS package

## Setup

0. Follow the instructions in [millrace_moveit_config](https://github.com/osuapplevision/millrace_moveit_config) to install this repository and it's dependencies.
1. Use the launch files to get started:
    * `roslaunch applevision_rospkg config.launch` for tf frames and camera info.
    * `roslaunch applevision_rospkg fake_sensor.launch` for fake simulated robot (calls `config.launch` automatically).
    * `roslaunch applevision_rospkg read_sensor_robot.launch` for real robot (calls `config.launch` automatically) (UNTESTED).
