#!/usr/bin/env python

import rospy
from std_msgs.msg import *

def callback(msg):
    if msg.data == 4:
        pub = rospy.Publisher('msg_to_receiver', String, queue_size=1)

        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            pub.publish('my name is fourth')
            rate.sleep()


rospy.init_node('fourth')
sub = rospy.Subscriber('start_ctl', Int32, callback, queue_size=1)
rospy.spin()
