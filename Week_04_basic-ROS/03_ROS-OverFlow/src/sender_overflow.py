#!/usr/bin/env python

import time
import sys

import rospy
from std_msgs.msg import Int32

rospy.init_node('sender')
pub = rospy.Publisher('my_topic', Int32)

rate = rospy.Rate(1000)
count = 1

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()
