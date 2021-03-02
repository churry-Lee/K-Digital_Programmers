#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, math
import cv2, time, rospy
import numpy as np
# AR tag의 거리/자세 정보 토픽 메세지
from ar_track_alvar_msgs.msg import AlvarMarkers
# Quaternion 값을 euler 값으로 변환
from tf.transformations import euler_from_quaternion
# 자동차 구동제어 토픽 메세지
from std_msgs.msg import Int32MultiArray

# arData 값 초기화
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0


def callback(msg):
    global arData

    for i in msg.markers:
        # AR tag의 x, y, z 좌표 저장 변수
        arData["DX"] = i.pose.pose.position.x
        arData["DY"] = i.pose.pose.position.y
        arData["DZ"] = i.pose.pose.position.z
        # AR 태그의 자세 정보(Quaternion) 저장 변수
        arData["AX"] = i.pose.pose.orientation.x
        arData["AY"] = i.pose.pose.orientation.y
        arData["AZ"] = i.pose.pose.orientation.z
        arData["AW"] = i.pose.pose.orientation.w


def start():
    rospy.init_node('ar_drive_info')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size =1 )
    xycar_msg = Int32MultiArray()


    while not rospy.is_shutdown():
        # AR tag의 위치/자세 정보Quaternion 값을 euler 값으로 변환
        (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"],
                                                arData["AZ"], arData["AW"]))
        # radian 값을 degree로 변환
        roll = math.degrees(roll)
        pitch = math.degrees(pitch)
        yaw = math.degrees(yaw)

        print("=======================")
        print(" roll  : " + str(round(roll,1)))
        print(" pitch : " + str(round(pitch,1)))
        print(" yaw   : " + str(round(yaw,1)))

        print(" x : " + str(round(arData["DX"],0)))
        print(" y : " + str(round(arData["DY"],0)))
        print(" z : " + str(round(arData["DZ"],0)))

        img = np.zeros((100, 500, 3))

        img = cv2.line(img, (25, 40), (25, 90), (0, 0, 255), 3)
        img = cv2.line(img, (250, 40), (250, 90), (0, 0, 255), 3)
        img = cv2.line(img, (475, 40), (475, 90), (0, 0, 255), 3)
        img = cv2.line(img, (25, 65), (475, 65), (0, 0, 255), 3)

        point = int(arData["DX"]) + 250

        if point > 475:
            point = 475
        elif point < 25:
            point = 25

        img = cv2.circle(img, (point, 65), 15, (0, 255, 0), -1)

        # x, y 좌표를 가지고 AR tag까지의 거리 계산(피타고라스)
        distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DY"], 2))
        distance = "{} Pixel".format(int(distance))
        cv2.putText(img, distance, (350, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        # DX, DY, Yaw 값을 문자열로 만들고 좌측 상단에 표시
        dx_dy_yaw = "DX: {}, DY: {}, Yaw: {}".format(int(arData["DX"]), int(arData["DY"]), round(yaw, 2))
        cv2.putText(img, dx_dy_yaw, (20, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255))

        cv2.imshow('AR tag Position', img)
        cv2.waitKey(10)

        angle = 20
        speed = 5

        xycar_msg.data = [angle, speed]
        motor_pub.publish(xycar_msg)
        
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start()
