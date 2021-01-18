#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32


def callback(msg):
    print(msg.data)

rospy.init_node('receiver')
sub = rospy.Subscriber('msg_send', Int32, callback, queue_size =10)
rospy.spin()
