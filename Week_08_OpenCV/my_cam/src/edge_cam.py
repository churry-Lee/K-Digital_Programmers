#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('cam_tune', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

while not rospy.is_shutdown():
    if cv_image.size != (640*480*3):
        continue
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70)

    cv2.imshow("original", cv_image)
    cv2.imshow("gray", gray)
    cv2.imshow("gaussian blur", blur_gray)
    cv2.imshow("edge", edge_img)
    cv2.waitKey(1)
