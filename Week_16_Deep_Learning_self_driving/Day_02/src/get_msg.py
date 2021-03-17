#!/usr/bin/env python

import rospy
from xycar_motor.msg import xycar_motor
from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge

import csv, time

name = 'data_get'
video_name = '/home/ch/Desktop/test.mkv'
csv_name = '/home/ch/Desktop/test.csv'

curr_angle = 99999
bridge = CvBridge()
cv_image = np.empty(shape=[0])

def motor_callback(data):
    global curr_angle

    curr_angle = data.angle

def img_callback(data):
    global cv_image
    global bridge
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node(name, anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
rospy.Subscriber("/xycar_motor", xycar_motor,  motor_callback)

out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (640, 480))

f = open(csv_name, 'w')
wr = csv.writer(f)
wr.writerow(['ts_micro', 'frame_index', 'wheel'])

while True:
    if current_angle == 99999:
        continue
    if cv_image.size != (640 * 480 * 3):
        continue
    break

cnt = 0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    
    cv2.imshow('image', cv_image)
    cv2.waitKey(1)
    wr.writerow([time.time(), cnt, curr_angle])
    out.write(cv_image)
    rate.sleep()

    if cnt == 4000:
        break
    
    cnt += 1

out.release()
f.close()
