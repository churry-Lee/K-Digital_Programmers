#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

rospy.init_node('sender')
pub = rospy.Publisher('msg_send', Int32, queue_size = 10)

rate = rospy.Rate(10)

for i in range(1, 100+1):
    rate.sleep()
    pub.publish(i)
