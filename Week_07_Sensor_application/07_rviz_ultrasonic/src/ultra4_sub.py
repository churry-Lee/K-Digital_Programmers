#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray

def callback(msg):
    print(msg.data)

rospy.init_node('ultra4_sub')
sub = rospy.Subscriber('ultra4', Int32MultiArray, callback)

rospy.spin()
