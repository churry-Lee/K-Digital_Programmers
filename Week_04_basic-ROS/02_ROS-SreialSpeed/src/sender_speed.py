#!/usr/bin/env python

import time
import sys

import rospy
from std_msgs.msg import String

rospy.init_node('sender')
pub = rospy.Publisher('speed', String)

rate = rospy.Rate(2)

send_msg = '#' * 5000000
send_msg_size = sys.getsizeof(send_msg)

while not rospy.is_shutdown():
    pub.publish(send_msg + ':' + str(time.time()) + ':' + \
            str(send_msg_size))
    rate.sleep()
