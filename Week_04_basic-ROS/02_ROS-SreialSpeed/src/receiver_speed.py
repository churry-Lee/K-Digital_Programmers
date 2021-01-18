#!/usr/bin/env python

import time

import rospy
from std_msgs.msg import *

def callback(msg):
    msgs = msg.data.split(':', 2)
    sec = time.time() - float(msgs[1])
    size = float(msgs[2])
    
    s = 'Size: %.2f byte, Time: %f sec'%(size, sec)
    speed = size / sec

    rospy.loginfo(s + ', ' + 'Send speed: ' + str(speed) + ' byte/sec')

rospy.init_node('receiver')
rospy.Subscriber('speed', String, callback)
rospy.spin()
