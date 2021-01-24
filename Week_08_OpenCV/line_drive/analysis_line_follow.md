```python
#!/usr/bin/env python

import cv2, time, rospy, math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
# setting var. xycar_motor() topic
motor_control = xycar_motor()
# setting CvBridge() module for using OpenCV at ROS system
bridge = CvBridge()
# setting init. array var. for saving subscribed image massage 
cv_image = np.empty(shape=[0])

# threshold for hsv
threshold = 40
width = 640
# for roi y coordinate
vertical = 350
# for scan and roi setting
scan_width, scan_height = 320, 20
# for sub roi of main_roi to detecting lane
area_width, area_height = 20, 10
# left scan: x coordinate 30 ~ 320
#            y coordinate 300 ~ 320
lmid = scan_width
# right scan: x coordinate 320 ~ 610
#             y coordinate 300 ~ 320
rmid = width - scan_width
# (20 - 10) // 2 == 5
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
hsv_pixel_threshold = 0.5 * area_width * area_height
# setting init angle & speed
angle, speed = 0, 0
# setting def. for publishing
def motor_pub(angle, speed):
    global pub
    global motor_control

    motor_control.angle = angle
    motor_control.speed = speed
    
    while pub.get_num_connections() == 0:
        continue
    # publish to xycar_motor()
    pub.publish(motor_control)
# setting subsribed call back def.
def img_callback(data):
    global cv_image
    
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
# setting node
rospy.init_node('cam_tune')
# setting subscriber topic
rospy.Subscriber('/usb_cam/image_raw/', Image, img_callback, queue_size=1)
# setting publisher topic
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
    if cv_image.size != (640*480*3):
        continue
    # draw yellow squre on the cv_image
    origin = cv2.rectangle(cv_image, (0, vertical),\
                          (width, vertical + scan_height),\
                           (0, 255, 255), 3)
    # convert BGR to GrayScale
    gray = cv2.cvtColor(origin, cv2.COLOR_BGR2GRAY)
    # brightness - 100 
    gray = cv2.subtract(gray, 100)
    # convert BGR
    gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    # GaussianBlur filter
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    # BGR to HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    # low-bound
    lbound = np.array([0, 0, threshold], dtype=np.uint8)
    # upper-bound
    ubound = np.array([10, 10, 255], dtype=np.uint8)
    # binarization
    bin_hsv = cv2.inRange(hsv, lbound, ubound)
    cvt_hsv = cv2.cvtColor(bin_hsv, cv2.COLOR_GRAY2BGR)

    # find lane at sub_roi using hsv and canny
    # main_roi setting and image transformating
    hsv_roi = bin_hsv[vertical:vertical + scan_height, : ]
    # hsv
    left, right = -1, -1
    # lane detecting at left sub_roi
    for l in range(lmid, 30, -1):
        # hsv sub_roi
        area = hsv_roi[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > hsv_pixel_threshold:
            left = l
            break
    # lane detecting at right sub_roi
    for r in range(rmid, width-30):
        # hsv sub_roi
        area = hsv_roi[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > hsv_pixel_threshold:
            right = r
            break
    # left hsv green squre
    if left != -1:
        # if left lane detecting success, draw squre
        hsv_lsquare = cv2.rectangle(origin, (left-area_width, vertical+row_begin),\
                                (left, vertical+row_end),\
                                (0, 255, 0), 6)
    else:
        print("Lost left line")
    # right hsv green squre
    if right != -1:
        # if right lane detecting, draw squre
        hsv_rsquare = cv2.rectangle(origin, (right, vertical+row_begin),\
                                (right+area_width, vertical+row_end),\
                                (0, 255, 0), 6)
    else:
        print("Lost right line")
    # center of detected lane
    center_line = (left + right) // 2
    # center line display
    origin = cv2.line(origin, (center_line, vertical-20),\
                                (center_line-40, vertical+20), (0, 0, 255), 3)
    origin = cv2.line(origin, (center_line, vertical-20),\
                                (center_line+40, vertical+20), (0, 0, 255), 3)

    cv2.imshow('original', origin)
    #cv2.imshow('roi', hsv_roi)
    #cv2.imshow('hsv', cvt_hsv)
    speed = 3
    # caculating where is center_line
    pixel_diff = 320 - center_line
    # if center line on the left
    if pixel_diff != 0:
        angle = 0.25 * pixel_diff
    # if center line equal my pose
    else:
        angle = 0
    # call the motor_pub def.
    motor_pub(angle, speed)
    # setting frame rate
    # 3 means 30 ms(mili-second) => 0.03 sec
    # therefor, 1 frame / 0.03 sec => about 33 fps
    cv2.waitKey(30)
```
