#!/usr/bin/env python

from __future__ import print_function
from re import L

import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import tf
from std_msgs.msg import String, Header
from sensor_msgs.msg import Range
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber

from applevision_rospkg.msg import PointWithCovarianceStamped, RegionOfInterestWithCovarianceStamped

class SynchronizerMinTick(ApproximateTimeSynchronizer):
    def __init__(self, fs, queue_size, slop, min_tick):
        super(SynchronizerMinTick, self).__init__(fs, queue_size, slop)
        self.min_tick = rospy.Duration.from_sec(min_tick)
        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)
    
    def signalMessage(self, *msg):
        if self.tick_timer:
            self.tick_timer.shutdown()

        self.last_tick = rospy.Time()
        self.tick_timer = rospy.Timer(self.min_tick, self._timerCallback, reset=True, oneshot=True)
        return super(SynchronizerMinTick, self).signalMessage(*msg)

    def _timerCallback(self, _):
        # signalMessage with whatever the latest data is
        with self.lock:
            found_msg = None
            for q in self.queues:
                if not q:
                    continue
                highest_stamp = max(q)
                if rospy.Time.now() - highest_stamp <= self.min_tick:
                    found_msg = q[highest_stamp]
            
            if not found_msg:
                # nothing has pinged within the tick rate
                rospy.logwarn('SynchronizerMinTick: no packets within min tick rate')
                ret = [None for _ in range(len(self.queues))]
            else:
                target_stamp = found_msg.header.stamp
                # collect all things that have pinged within the slop
                ret = []
                for q in self.queues:
                    if not q:
                        ret.append(None)
                        continue
                    best_stamp_diff, best_stamp = min((abs(target_stamp - s), s) for s in q)
                    if best_stamp_diff <= self.slop:
                        ret.append(q[best_stamp])
                    else:
                        ret.append(None)
            
            # empty queue and return
            self.signalMessage(*ret)
            for q in self.queues:
                q.clear()

def test_callback(*msgs):
    out = 'args: '
    for i, m in enumerate(msgs):
        if m:
            out += str(i)
    print(out)


if __name__ == '__main__':
    rospy.init_node('applevision_test')

    print('starting!')

    dist = Subscriber('applevision/apple_dist', Range)
    camera = Subscriber('applevision/apple_camera',
                        RegionOfInterestWithCovarianceStamped)
    sync = SynchronizerMinTick([dist, camera], 10, 0.017, 1)
    sync.registerCallback(test_callback)


    distpub = rospy.Publisher('applevision/apple_dist', Range, queue_size=10)
    campub = rospy.Publisher('applevision/apple_camera', RegionOfInterestWithCovarianceStamped, queue_size=10)

    def test_routine(_):
        for _ in range(10):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp)
            distpub.publish(Range(header=header))
            campub.publish(RegionOfInterestWithCovarianceStamped(header=header))
            rospy.sleep(0.8)
        
        for _ in range(10):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp)
            campub.publish(RegionOfInterestWithCovarianceStamped(header=header))
            rospy.sleep(0.8)

        for _ in range(10):
            stamp = rospy.Time.now()
            header = Header(stamp=stamp)
            distpub.publish(Range(header=header))
            rospy.sleep(0.8)

    pubtimer = rospy.Timer(rospy.Duration.from_sec(0.1), test_routine, oneshot=True)

    rospy.spin()

