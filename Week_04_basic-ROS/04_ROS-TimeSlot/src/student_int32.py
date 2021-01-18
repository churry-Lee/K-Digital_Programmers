#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def callback(msg):
    print(msg.data)

rospy.init_node('student')
sub = rospy.Subscriber('msg_to_students', Int32, callback, queue_size=1)
rospy.spin()
