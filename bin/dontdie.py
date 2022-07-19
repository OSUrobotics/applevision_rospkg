#!/usr/bin/env python2

# How successful is applevision_motion? 
# Goal: run applevision_motion several times & determine success or failure

# Before starting: have rviz & simulated camera open
    # roslaunch applevision_moveit_config demo.launch
    # roslaunch applevision_rospkg fake_sensor.launch

# To (hopefully) run this:
    # src/applevision_rospkg/bin/dontdie.py

import logging
Logger = logging.getLogger(name="name")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

import applevision_motion
import move_group
import functions
import rospy

# initial position
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [0, -.58, -2.26, -.44, 1.62, .74]
#apple coords
apple = [-.51, -.16, 1.3]

Logger.debug("hi")

#rospy.init_node('applevision_motion')

#idk it's getting stuck here
bot = move_group.MoveGroupInterface()

Logger.debug("hi")

# if you want to get fancy, make a list of trials + results and log to a csv

num = int(input("Run how many times? "))

Logger.debug(functions.getpose())

# loops through given number of times
for x in range(num):
    # reset
    result = "fail"

    # go to home position
    bot.moveToJointPosition(joints, initial)
    Logger.debug("move joints done")

    # motion runthrough
    applevision_motion.main()
    Logger.debug("main done")

    # check final position
    final = functions.getpose()
    Logger.debug(final)

    # should probably edit this so the "nearby" is actually nearby
    # determine success
    if functions.nearby(final, apple) == True:
        result = "success"

    x+=1
    # log results (see note above)
    print("Number " + str(x) + " was a " + result)
    
# if __name__ == '__main__':
#     rospy.init_node('applevision_motion')