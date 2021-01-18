#!/usr/bin/env python

import rospy
from std_msgs.msg import *

def callback(msg):
    s = msg.data
    rospy.loginfo(s)

rospy.init_node('Receiver')
pub = rospy.Publisher('start_ctl', Int32, queue_size=1)
sub = rospy.Subscriber('msg_to_receiver', String, callback)

rate = rospy.Rate(0.5)

while not rospy.is_shutdown():
    for i in range(4+1):
        pub.publish(i)
        rate.sleep()
