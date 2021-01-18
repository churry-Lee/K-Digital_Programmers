#!/usr/bin/env python

import rospy
from msg_send.msg import my_msg
from std_msgs.msg import String

done = False

def callback(data):
    print(data.data)
    done = True

rospy.init_node('remote_student', anonymous=True)
pub = rospy.Publisher('msg_to_xycar', my_msg, queue_size=1)
rospy.Subscriber('msg_from_xycar', String, callback)

rate = rospy.Rate(1)

msg = my_msg()
msg.first_name = "Gil-Dong"
msg.last_name = "Hong"
msg.age = 15
msg.score = 100
msg.id_number = 12345678
msg.phone_number = "010-1234-5678"

while not rospy.is_shutdown() and not done:
    pub.publish(msg)
    print("sending message...")
    rate.sleep()
