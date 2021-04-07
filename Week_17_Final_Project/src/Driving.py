#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2, math, time
import numpy as np
from collections import deque

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
# import csv

frame = np.empty(shape=[0])
cap = cv2.VideoCapture(0)
height, width = 480, 640
mid_x = 320

# video_name = '/home/nvidia/Desktop/test.mkv'
# csv_name = '/home/nvidia/Desktop/test.csv'
# curr_angle = 99999

speed = 50
max_steer = 40
steering_offset = 7

prev_lx = 0
prev_rx = 0
q = deque(maxlen=5)

# =============== Hough Transform =================
def calculate_lines(img, lines):
    global left_line, right_line, both_line

    left = []
    right = []

    try:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)
            # print("x1: {}, y1: {}, x2: {}, y2: {}".format(x1, y1, x2, y2))
            parameters = np.array([0, 0])
            if x1 != x2:
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                # print("slope: {}".format(parameters[0]))
            else:
                continue
            slope = parameters[0]
            y_intercept = parameters[1]
            if x2 < 320:
                left.append((slope, y_intercept))
            elif 320 < x2:
                right.append((slope, y_intercept))

        if len(left) != 0:
            left_avg = np.average(left, axis=0)
            left_line = calculate_coordinates(img, left_avg)
        elif len(left) == 0:
            left_line = np.array([0, 0, 0, 0])

        if len(right) != 0:
            right_avg = np.average(right, axis=0)
            right_line = calculate_coordinates(img, right_avg)
        elif len(right) == 0:
            right_line = np.array([0, 0, 0, 0])

        if len(left) != 0 or len(right) != 0:
            both = left + right
            both_avg = np.average(both, axis=0)               # [slope_avg, y_intercept_avg]
            both_line = calculate_coordinates(img, both_avg)  # [x1, y1, x2, y2]
        elif len(left) == 0 and len(right) == 0:
            both_line = np.array([320, 480, 320, 280])

        return np.array([left_line, right_line]), np.array([both_line])
    
    except TypeError:
        pass

def calculate_coordinates(img, parameters):
    global height

    slope, intercept = parameters
    
    y1 = height
    y2 = int(y1 - 100)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return np.array([x1, y1, x2, y2])

def visualize_direction(img, lines):
    lines_visualize = np.zeros_like(img)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            try:
                cv2.line(lines_visualize, (320, y1), (x2+(320-x1), y2), (0, 0, 255), 5)
            except OverflowError:
                pass

    return lines_visualize

def visualize_lines(img, lines):
    lines_visualize = np.zeros_like(img)
    if lines is not None:
        for x1, y1, x2, y2 in lines:
            try:
                cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 5)
            except OverflowError:
                pass

    return lines_visualize

# =============== Image Processing =================
def perspective_img(img):
    global frame
    global height, width, mid_x
    
    point_1 = [130, height-200]
    point_2 = [20, height- 160]
    point_3 = [width-130, height-200]
    point_4 = [width-20, height-160]

    # draw area
    area = np.zeros_like(img)
    area = cv2.line(area, tuple(point_1), tuple(point_2), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_3), tuple(point_4), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_1), tuple(point_3), (255, 255, 0), 2)
    area = cv2.line(area, tuple(point_2), tuple(point_4), (255, 255, 0), 2)
    area = cv2.line(area, (320, 480), (320, 0), (255, 255, 0), 4)

    warp_src  = np.array([point_1, point_2, point_3, point_4], dtype=np.float32)
    
    warp_dist = np.array([[0,0],\
                          [0,height],\
                          [width,0],\
                          [width, height]],\
                         dtype=np.float32)

    M = cv2.getPerspectiveTransform(warp_src, warp_dist)
    Minv = cv2.getPerspectiveTransform(warp_dist, warp_src)
    warp_img = cv2.warpPerspective(img, M, (width, height), flags=cv2.INTER_LINEAR)
    
    return warp_img, M, Minv, area

def set_roi(img):
    global height, width, mid_x

    region_1 = np.array([[
        (20, height-20),
        (20, height-150),
        (200, height-210),
        (mid_x-20, height-210),
        (mid_x-200, height-20)
    ]])

    region_2 = np.array([[
        (width-10, height-20),
        (width-10, height-150),
        (width-200, height-210),
        (mid_x+20, height-210),
        (mid_x+200, height-20)
    ]])
    
    # region_2 = np.array([[
    #     (20, height-20),
    #     (20, height-150),
    #     (200, height-210),
    #     (width-200, height-210),
    #     (width-10, height-150),
    #     (width-10, height-20),
    # ]])
    
    mask = np.zeros_like(img)
    left_roi = cv2.fillPoly(mask, region_1, 255)
    right_roi = cv2.fillPoly(mask, region_2, 255)
    roi = cv2.bitwise_and(img, mask)
    
    return roi

def canny_edge(img, low_th, high_th):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    canny = cv2.Canny(blur, low_th, high_th)
    return canny

def lane_keeping(lines, direction, Kp, k):
    global height, width
    global speed, max_steer, steering_offset
    global prev_lx, prev_rx, q

    lane_width = 540
    mid = 360
    mean_lx = (lines[0][0] + lines[0][2]) // 2
    mean_rx = (lines[1][0] + lines[1][2]) // 2
    if abs(prev_lx - mean_lx) < abs(prev_rx - mean_rx):
        prev_lx = mean_lx
        prev_rx = mean_lx + lane_width
        trust = "left"
    else:
        prev_rx = mean_rx
        prev_lx = mean_rx - lane_width
        trust = "right"
    cte = mid - (prev_rx + prev_lx) // 2 
    cte_term = abs(math.atan2(k * cte, 10))
    
    x1, y1, x2, y2 = direction[0]
    parameters = np.polyfit((x1, x2), (y1, y2), 1)
    q.append(abs(parameters[0]))
    slope_avg = sum(q) / len(q)
    angle = math.atan2(1, slope_avg) + cte_term
    angle = math.degrees(angle)

    if 0 < parameters[0]:
        steer = -angle * Kp
    elif parameters[0] < 0 :
        steer = angle * Kp
    steer = np.clip(steer, -max_steer, max_steer)

    return steer+7, speed 

def img_callback(data):
    global frame, bridge

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

def main():
    global frame, bridge
    global height, width, mid_x
    global prev_lx, prev_rx

    bridge = CvBridge()
    motor_control = xycar_motor()

    rospy.init_node('lane_detect')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback, queue_size=1)
    pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

    # out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (640, 480))
    # f = open(csv_name, 'w')
    # wr = csv.writer(f)
    # wr.writerow(['ts_micro', 'frame_index', 'wheel'])

    while not rospy.is_shutdown():
        # if curr_angle == 99999:
        #     continue
        if frame.size != (640 * 480 * 3):
            continue
        height, width = frame.shape[0:2]
        mid_x = width // 2
        
        #============== image transform ==============
        canny = canny_edge(frame, 50, 150)
        roi = set_roi(canny)
        warp_img, M, Minv, area = perspective_img(roi)
        #============== Hough Line Transform ==============
        hough = cv2.HoughLinesP(warp_img, 1, np.pi/180, 120, np.array([]), minLineLength = 10, maxLineGap = 20)
        if hough is not None:
            lines, direction = calculate_lines(warp_img, hough)
        
        warp_img = cv2.cvtColor(warp_img, cv2.COLOR_GRAY2BGR)
        lines_visualize = visualize_lines(warp_img, lines)
        warp_img = cv2.addWeighted(warp_img, 0.9, lines_visualize, 1, 1)
        direction_visualize = visualize_direction(warp_img, direction)
        warp_img = cv2.addWeighted(warp_img, 0.9, direction_visualize, 1, 1)
        warp_img = cv2.circle(warp_img, (int(prev_lx), 240), 5, (255, 0, 0), -1)
        warp_img = cv2.circle(warp_img, (int(prev_rx), 240), 5, (255, 0, 0), -1)
        roi = cv2.addWeighted(roi, 0.9, area, 1, 1)
        
        cv2.imshow('warp', warp_img)
        cv2.imshow('result', roi)
        Kp = 1.1
        k = 0.005
        
        delta, v = lane_keeping(lines, direction, Kp, k)
        
        motor_control.angle = delta
        motor_control.speed = v
        pub.publish(motor_control)

        # wr.writerow([time.time(), cnt, curr_angle])
        # out.write(roi)
        cv2.waitKey(1)

if __name__ == "__main__":
    try:
        main()
    finally:
        cap.release()
        cv2.destroyAllWindows()
