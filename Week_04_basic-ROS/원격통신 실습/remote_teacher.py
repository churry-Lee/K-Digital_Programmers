#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg
from std_msgs.msg import String
import time

def callback(data):
    st_name = data.last_name + '' + data.first_name
    curr_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
    st_name2 = 'Good morning, ' + st_name + '' + curr_time

    pub.publish(st-name2)

rospy.init_node('remote_teacher', anonymous=True)

pub = rospy.Publisher('msg_from_xycar', String, queue_size=1)
sub = rospy.Subscriber('msg_to_xycar', my_msg, callback)

rospy.spin()
