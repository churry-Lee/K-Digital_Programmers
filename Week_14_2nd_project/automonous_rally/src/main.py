#!/usr/bin/env python

import cv2, time
from pyzbar import pyzbar
from ros_module import *
from dqn_drive import *
from parking import *
from ultrasonic_drive import *
from yolo_drive import *
from go_backward import *
from std_msgs.msg import Int32MultiArray
import rospy

dqn_name = "jdy1"
pth_path = "/home/nvidia/automonous_rally/src/rally/src"

if dqn_name == "jdy1":
	input_size = 6
	hidden_layers = [16,16]
	output_size = 3
	pth_num = 153
	sonar_max = 180.0
elif dqn_name == "jdy2":
	input_size = 6
	hidden_layers = [32, 32, 32]
	output_size = 9
	pth_num = 323202
	sonar_max = 120.0
elif dqn_name == "xytron":
	input_size = 7
	hidden_layers = [256,256,256,256]
	output_size = 9
	pth_num = 349
	sonar_max = 70.0
else:
	input_size = 5
	hidden_layers = [512,512,512,512]
	output_size = 5
	pth_num = 349
	sonar_max = 120.0

rm = ROS("team_name")
rate = rospy.Rate(10)
angle = 0
speed = 0

distance=[0,0,0,0,0,0,0,0]
def Ultra_callback(data):
	global distance 
	distance = data.data
SonarSub = rospy.Subscriber('xycar_ultrasonic', Int32MultiArray,Ultra_callback,queue_size=1)

obj = {
	"3 dqn dqn_drive_start":DQN(input_size, hidden_layers, output_size, pth_num, pth_path, sonar_max),
	"9 ar parking":PARK(),
	"1 algorithm drive_avoid_obstacle":ULTRASONIC(),
	"5 yolo bottle":YOLO(),
	"6 yolo pottedplant":YOLO(),
	"2 algorithm drive_turn_back":BACKWARD()
}
mode = "1 algorithm drive_avoid_obstacle"
while rm.get_shutdown():
	camera_image = rm.get_camera_image_data()
	if not camera_image.size == (640*480*3):
		continue

	qrcodes = pyzbar.decode(camera_image)
	for qtcode in qrcodes:
		qr_string = qtcode.data
		if qr_string in obj.keys():
			mode = qr_string
	print(mode)
	if mode == "":
		rm.set_motor(0, 0)
		continue

	obj[mode].set_motor(angle, speed)

	if mode == "3 dqn dqn_drive_start":
		obj[mode].set_data([rm.get_ultrasonic_data(), angle], dqn_name=dqn_name)
	elif mode == "9 ar parking":
		obj[mode].set_data(rm.get_ar_tags_datas())
	elif mode == "5 yolo bottle" or mode == "6 yolo pottedplant":
		obj[mode].set_data([qrcodes, rm.get_darknet_bounding_boxes(), rm.get_ultrasonic_data()])
	elif mode == "1 algorithm drive_avoid_obstacle":
		obj[mode].set_data(rm.get_ultrasonic_data())
	elif mode == "2 algorithm drive_turn_back":
		obj[mode].set_data(rm.get_ultrasonic_data())
	else:
		rm.set_motor(0, 0)
		continue
	print(rm.get_ultrasonic_data())
	angle, speed = obj[mode].get_motor() 
	print("angle: {}, speed: {}".format(angle, speed))
	rm.set_motor(angle, speed)
	rate.sleep()
	
	
	
