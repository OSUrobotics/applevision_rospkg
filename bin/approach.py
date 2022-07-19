#!/usr/bin/env python2

import logging
Logger = logging.getLogger(name="name")
handler = logging.StreamHandler()
Logger.addHandler(handler)
Logger.setLevel(logging.DEBUG)

from math import sqrt
import rospy
import applevision_motion
import tf
from tf.listener import TransformListener
import rospy

# copy pasta from somewhere... but I dont know where TT
# gets position of palm
def getpose():
    #rospy.init_node('applevision_motion')
    broadcaster = tf.TransformBroadcaster()
    now = rospy.Time.now()
    sbr = tf.TransformBroadcaster()
    R = rospy.Rate(150)
    # while not rospy.is_shutdown():
    broadcaster.sendTransform((1, 1, 1), (0, 0, 0, 1), rospy.Time(), '/world','/palm')      
    R.sleep()

    listener = tf.TransformListener()
    now = rospy.Time.now()
    listener.waitForTransform('/world','/palm',rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform('/world', '/palm', rospy.Time(0))
    return(trans)

# calculating distance between apple and endeffector
# takes two arrays ([x,y,z]) for positions of apple and endeffector 
def nearby(real, apple):
    dstance = sqrt((abs(real[0]-apple[0]))**2+(abs(real[1]-apple[1]))**2+(abs(real[2]-apple[2]))**2)
    heightdiff = abs(real[2]-apple[2])
    # for now these are arbitrary numbers... will need to check what is acceptable
    if dstance < .3 and heightdiff <.05:
        return True

# initial position
joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
initial = [0, -.58, -2.26, -.44, 1.62, .74]
#apple coords
apple = [-.51, -.16, 1.3]

Logger.debug("hi")

#rospy.init_node('applevision_motion')

#idk it's getting stuck here
bot = applevision_motion.MoveGroupInterface()

Logger.debug("hi")

# if you want to get fancy, make a list of trials + results and log to a csv

num = int(input("Run how many times? "))

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
    final = getpose()
    print(final)

    # should probably edit this so the "nearby" is actually nearby
    # determine success
    if nearby(final, apple) == True:
        result = "success"

    x+=1
    # log results (see note above)
    print("Number " + str(x) + " was a " + result)
    
# if __name__ == '__main__':
#     rospy.init_node('applevision_motion')