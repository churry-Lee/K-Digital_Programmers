#!/usr/bin/env python
#-*- coding: utf-8 -*-

import cv2, math, time
import numpy as np
from collections import deque

import rospy
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
from xycar_motor.msg import xycar_motor
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

import Driving, Avoiding, Parking, CrosswalkStop, stopline_canny

# video_name = '/home/nvidia/Desktop/final.mkv'

lap, b_lap = 1, 0   # lap 카운트
tmp, obstacle, stop, linecount = True, False, False, 1   # Flag 초기화
# 일반 주행 튜닝값
# Kp_std = 1.1     # 직진 코스
Kp_std = 1.2     # 직진 코스
Kp_std_s = 1.45   # s자 코스
# k_std_obs = 0.0008
k_std_obs = 0.0009
# k_std = 0.005
k_std = 0.006
std_low_th, std_high_th = 50, 150  # 직진 코스 canny 임계값
# s_low_th, s_high_th = 30, 100      # s자 코스 canny 임계값
s_low_th, s_high_th = 50, 100      # s자 코스 canny 임계값
# 장애물 주행 튜닝값
Kp_obs = 2.7
# k_obs = 0.005
k_obs = 0.006
# delta_obs = 25
delta_obs = 22
vel_obs = 34
# vel_obs = 35
lidar_dist_init = 0.7
lidar_dist_obs = 0.65
# 횡단보도 인식 임계값
stop_min, stop_max = 700, 900

# 주차에 사용되는 변수 초기화 
arData = {"id": 0, "bool": None,  "DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0
ultra_data = None
Kp_park = 2.5     # 주차장 진입
k_park = 0.001    # 주차장 진입
# dist_th = 49     # 후진 초음파 센서 거리 임계값
dist_th = 50     # 후진 초음파 센서 거리 임계값
parking_vel = 20  # 주차 속도
back_th = 0.01    # 최종 후진 시 AR tag와 정렬을 맞추기 위한 임계값

def park_end():
    global pub, motor_control
    global pitch, arData
    global parking_vel
    
    distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
    (_, pitch, _)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
    steer = 7    

    while 0.4 < distance:
        steer = math.atan2(arData["DX"], arData["DZ"])
        motor_control.angle = math.degrees(steer) * 5.0
        motor_control.speed = parking_vel
        pub.publish(motor_control)
        distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
    
    motor_control.angle = 7
    motor_control.speed = 0
    pub.publish(motor_control)
    exit()

def ar_parking(distance, dx, dy, yaw):
    global arData
    global pub, motor_control
    global ultra_data, dist_th, back_th

    steer_cal = yaw
    steer = 7
    car_yaw = math.atan2(dx, dy) - yaw
    sec = time.time()
    while time.time() - sec < 3.0:
        motor_control.angle = -(steer - math.degrees(steer_cal)) * 1.5
        motor_control.speed = parking_vel
        pub.publish(motor_control)

    while dist_th < ultra_data[7]:
        # if ultra_data[7] >= 60:
        #     print("===== 주의: {} =====".format(ultra_data[7]))
        # elif 50 <= ultra_data[7] < 60:
        #     print("===== 경고: {} =====".format(ultra_data[7]))
        # elif 0 <= ultra_data[7] < 10:
        #     print("===== 위험: {} =====".format(ultra_data[7]))
        # motor_control.angle = (steer - math.degrees(steer_cal)) * 1.2
        motor_control.angle = 50
        motor_control.speed = -parking_vel
        pub.publish(motor_control)
    while abs(car_yaw) > back_th:
        (_, pitch, _)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        car_yaw = math.atan2(arData["DX"], arData["DZ"]) - pitch
        # motor_control.angle = -math.degrees(car_yaw) * 3.0
        motor_control.angle = -50
        motor_control.speed = -parking_vel
        pub.publish(motor_control)
    park_end()
    pub.publish(motor_control)

def callback(msg):
    global arData, tmp, lap, b_lap
    ar_bool = False
    ar_bool_1 = False
    for i in msg.markers:
        if i.id == 2 and b_lap > 3:
            ar_bool = True
            arData["id"] = i.id
            arData["DX"] = i.pose.pose.position.x
            arData["DY"] = i.pose.pose.position.y
            arData["DZ"] = i.pose.pose.position.z
            arData["AX"] = i.pose.pose.orientation.x
            arData["AY"] = i.pose.pose.orientation.y
            arData["AZ"] = i.pose.pose.orientation.z
            arData["AW"] = i.pose.pose.orientation.w
        if i.id == 1:
            ar_bool_1 = True
    if ar_bool_1 == True and b_lap != lap:
        tmp = True
        b_lap += 1
    arData["bool"] = ar_bool
    # print(msg)

frame = np.empty(shape=[0])
cap = cv2.VideoCapture(0)

def img_callback(data):
    global frame, bridge

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

obs_dist = [0 for _ in range(505)]

def lidar_callback(data):
    global obs_dist
    
    obs_dist = data.ranges

def ultra_callback(data):
    global ultra_data

    ultra_data = data.data

def main():
    global pub, motor_control
    global frame, bridge, height, width, mid_x
    global arData, obs_dist
    global linecount, obstacle, tmp, stop
    global Kp_std, Kp_std_s, k_std, Kp_obs, k_obs, delta_obs, vel_obs, Kp_park, k_park, k_std_obs
    global std_low_th, std_high_th, s_low_th, s_high_th
    global lap, b_lap, stop_min, stop_max
    global lidar_dist_init, lidar_dist_obs

    bridge = CvBridge()
    motor_control = xycar_motor()
    
    rospy.init_node('drive')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback, queue_size=1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, ultra_callback, queue_size=1)
    pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size=1)

    # out = cv2.VideoWriter(video_name, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (640, 480))
    
    start_time = time.time()
    stop_time = time.time()
    while not rospy.is_shutdown():
        if frame.size != (640*480*3):
            continue
        height, width = frame.shape[0:2]
        mid_x = width // 2

        # ========== Image Processing ==========
        if tmp == False:
            canny = Driving.canny_edge(frame, s_low_th, s_high_th)
        elif tmp == True:
            canny = Driving.canny_edge(frame, std_low_th, std_high_th)
        roi = Driving.set_roi(canny)
        warp_img, M, Minv, area = Driving.perspective_img(roi)
        hough = cv2.HoughLinesP(warp_img, 1, np.pi/180, 90, np.array([]), minLineLength=50, maxLineGap=20)
        if hough is not None:
            lines, direction = Driving.calculate_lines(warp_img, hough)
        # print("lap: {}".format(lap))
        if b_lap > 3:
            print("----- Find Parking Lot -----")
            distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
            # print("distance: {}", distance)
            (roll, pitch, yaw)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
            if 0.6 < distance <= 1.7:
                steer = math.atan2(arData["DX"], arData["DZ"])
                motor_control.angle = math.degrees(steer) * 1.5
                motor_control.speed = parking_vel
            elif 0.1 < distance <= 0.6:
                ar_parking(distance, arData["DX"], arData["DZ"], pitch)
            else:
                delta, v = Driving.lane_keeping(lines, direction, Kp_park, k_park)
                motor_control.angle = delta
                motor_control.speed = 30
            print("Distance: {}".format(distance))
            pub.publish(motor_control)
        else:
            # ========== crosswalk stop ==========
            stop, image, cnt_nzero = stopline_canny.detect_stopline(frame, stop_min, stop_max, stop_time)
            if stop == True:
                print("----- Crosswalk stop -----")
                tmp_time = time.time()
                while time.time() - tmp_time <= 5:
                    motor_control.angle = 7
                    motor_control.speed = 0
                    pub.publish(motor_control)
                stop_time = time.time()
                tmp = False
                obstacle = False
                lap += 1
            
            # ========== lidar avoid drive ==========
            if obstacle == False:
                [lidar_obs_count, lidar_count_l, lidar_count_r] = Avoiding.obstacle_cnt(obs_dist, lidar_dist_init, 40)
                # [lidar_obs_count, lidar_count_l, lidar_count_r] = Avoiding.obstacle_cnt(obs_dist, lidar_dist_init, 20)
            elif obstacle == True:
                [lidar_obs_count, lidar_count_l, lidar_count_r] = Avoiding.obstacle_cnt(obs_dist, lidar_dist_obs, 55)
            if lidar_count_l > lidar_count_r + 5 and tmp == True:
                motor_control.angle = delta_obs
                motor_control.speed = vel_obs
                obstacle = True
            elif lidar_count_r > lidar_count_l + 5 and tmp == True:
                motor_control.angle = -delta_obs
                motor_control.speed = vel_obs
                obstacle = True
            else:
                # ========== standard drive ==========
                if obstacle == False:
                    if tmp == True:
                        delta, v  = Driving.lane_keeping(lines, direction, Kp_std, k_std_obs)
                    elif tmp == False:
                        delta, v = Driving.lane_keeping(lines, direction, Kp_std, k_std)
                # ========== lidar avoid drive ==========
                elif obstacle == True and tmp == True:
                    delta, _ = Driving.lane_keeping(lines, direction, Kp_obs, k_obs)
                    v = vel_obs
                motor_control.angle = delta
                motor_control.speed = v
            pub.publish(motor_control)
        
        # print("lines: {}".format(lines))
        # print("non_zero: {}".format(cnt_nzero))
        # cv2.imshow("L", warp_img)
        # print("tmp: {}, obstacle: {}, lap: {}, b_lap: {}".format(tmp, obstacle, lap, b_lap))
        # out.write(frame)
        # cv2.waitKey(1)
            
if __name__ == "__main__":
    main()
