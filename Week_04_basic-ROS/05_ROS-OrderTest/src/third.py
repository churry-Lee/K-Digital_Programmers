#!/usr/bin/env python

import rospy
from std_msgs.msg import *

def callback(msg):
    if msg.data == 3:
        pub = rospy.Publisher('msg_to_receiver', String, queue_size=1)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            pub.publish('my name is third')
            rate.sleep()


rospy.init_node('third')
sub = rospy.Subscriber('start_ctl', Int32, callback, queue_size=1)
rospy.spin()
