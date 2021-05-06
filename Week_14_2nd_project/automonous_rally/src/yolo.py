#! /usr/bin/env python

import rospy
from ros_module import *
from xycar_motor.msg import xycar_motor

rm = ROS("jdy")
pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
angle = 0
speed = 0
msg = xycar_motor()
rate = rospy.Rate(10)
x = 320
while not rospy.is_shutdown():
	bbs = rm.get_darknet_bounding_boxes()
	for bb in bbs:
		if bb["class"] == "person":
			x = (bb["xmin"] + bb["xmax"]) // 2
			print x
	angle = (x - 320) // 4
	speed = 15
	print(bbs)
	msg.angle = angle
	msg.speed = speed
	pub.publish(msg)
	rate.sleep()
