#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import *

def callback(msg):
    for i in range(10000001):
        temp = i
        if temp == 10000000:
            s = "callback is being processed {}".format(msg.data)
            rospy.loginfo(s)
        else:
            continue

rospy.init_node('receiver')
rospy.Subscriber('my_topic', Int32, callback, queue_size=1)

rospy.spin()
