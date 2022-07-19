#!/usr/bin/env python2
#for testing random things

from math import sqrt
import rospy
import applevision_motion
import tf
from tf.listener import TransformListener

# copy pasta from somewhere... but I dont know where
# gets position of palm
# getpose() only works when it's not running in a loop with the approach and stuff...
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
    
#_________________________________________________________________
