#!/usr/bin/env python

import cv2, time, rospy, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor

motor_control = xycar_motor()
bridge = CvBridge()
cv_image = np.empty(shape=[0])

threshold = 100
width = 640
scan_width, scan_height = 200, 20
lmid = scan_width
rmid = width - scan_width
area_width, area_height = 20, 10
vertical = 430
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_threshold = 0.8 * area_height * area_width

mid = 320
lane_width = 50.0 # cm
cm_per_pixel = 0.09486004321925537 # cm

def motor_pub(angle, speed):
    global pub
    global motor_control

    motor_control.angle = angle
    motor_control.speed = speed
    
    while pub.get_num_connections() == 0:
        continue

    pub.publish(motor_control)


def img_callback(data):
    global cv_image
    
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('cam_tune')
rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
    if cv_image.size != (640*480*3):
        continue
        
    roi = cv_image[vertical:vertical+scan_height, :]
    frame = cv2.rectangle(cv_image, (0, vertical), (width, vertical + scan_height), (0, 255, 0), 3)
    frame = cv2.line(cv_image, (320, 200), (320, 480), (0, 0, 255), 3, cv2.LINE_4)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold], dtype=np.uint8)
    ubound = np.array([135, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1
    for l in range(0, lmid - area_width):
    #for l in range(area_width, lmid + 1):
        area = bin[row_begin:row_end, l:l + area_width]
        #area = bin[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > pixel_threshold:
            left = l
            break

    for r in range(width - area_width, rmid + 1, -1):
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_threshold:
            right = r
            break
    

    lane_pos = ((left+area_width) + right) / 2

    if left != -1:
        lsquare = cv2.rectangle(view, (left, row_begin), (left + area_width, row_end), (0, 255, 0), 3)
        lane_line = cv2.line(view, (lane_pos, row_begin), (lane_pos, row_end), (255, 0, 0), 3)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view, (right, row_begin), (right + area_width, row_end), (0, 255, 0), 3)
        lane_line = cv2.line(view, (lane_pos, row_begin), (lane_pos, row_end), (255, 0, 0), 3)
    else:
        print("Lost right line")
   
    mid_line = cv2.line(view, (320, row_begin), (320, row_end), (0, 0, 255), 3)
    mid_line = cv2.line(frame, (lane_pos, 200), (lane_pos, 480), (255, 0, 0), 3, cv2.LINE_4)

    cv2.imshow('origin', frame)
    cv2.imshow('view', view)
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    hsv = cv2.inRange(hsv, lbound, ubound)
    cv2.imshow('hsv', hsv)

    
    if (left != -1 or right != -1) and 320 < lane_pos:
        theta = math.atan(9.34 / (320 - lane_pos))
    elif (left != -1 or right != -1) and lane_pos < 320:
        theta = math.atan(9.34 / (320 - lane_pos))
    else:
        theta = 0
    
    angle = 2.5 * theta * 4

    motor_pub(angle, 3)

    cv2.waitKey(1)
