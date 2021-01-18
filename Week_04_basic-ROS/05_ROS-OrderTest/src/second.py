#!/usr/bin/env python

import rospy
from std_msgs.msg import *

def callback(msg):
    if msg.data == 2:
        pub = rospy.Publisher('msg_to_receiver', String, queue_size=0)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            pub.publish('my name is second')
            rate.sleep()


rospy.init_node('second')
sub = rospy.Subscriber('start_ctl', Int32, callback, queue_size=1)
rospy.spin()
