#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import cv2, time, math
import numpy as np
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion
from xycar_motor.msg import xycar_motor

# arData 값 초기화
arData = {"id": 0, "bool": None,  "DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0

def callback(msg):
    global arData
    
    ar_bool = False
    for i in msg.markers:
        if i.id == 2:
            ar_bool = True
            arData["id"] = i.id
            # AR tag의 x, y, z 좌표 저장 변수
            arData["DX"] = i.pose.pose.position.x
            arData["DY"] = i.pose.pose.position.y
            arData["DZ"] = i.pose.pose.position.z
            # AR 태그의 자세 정보(Quaternion) 저장 변수
            arData["AX"] = i.pose.pose.orientation.x
            arData["AY"] = i.pose.pose.orientation.y
            arData["AZ"] = i.pose.pose.orientation.z
            arData["AW"] = i.pose.pose.orientation.w
    arData["bool"] = ar_bool

def re_park():
    global pub, motor_control
    global distance, arData

    for _ in range(10):
        motor_control.angle = 50
        motor_control.speed = -20
        pub.publish(motor_control)
    
    car_yaw = math.atan2(arData["DX"], arData["DZ"]) - yaw
    while car_yaw > 0.01 :
        if car_yaw <= 0.01:
            break
        (_, yaw, _)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
        car_yaw = math.atan2(arData["DX"], arData["DZ"]) - yaw

        motor_control.angle = -50
        motor_control.speed = -speed
        pub.publish(motor_control)
    
    distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
    while 0.45 < distance:
        steer = math.atan2(arData["DX"], arData["DZ"]) - pitch
        motor_control.angle = math.degrees(steer) * 3.0
        motor_control.speed = 20
        pub.publish(motor_control)
        distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))

    exit()

def park_end(distance):
    global pub, motor_control
    global pitch, arData

    speed = 20
    while 0.45 < distance:
        steer = math.atan2(arData["DX"], arData["DZ"]) - pitch
        motor_control.angle = math.degrees(steer) * 3.0
        motor_control.speed = speed
        pub.publish(motor_control)
        distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
    
    motor_control.speed = 0
    pub.publish(motor_control)

    x_pos = arData["DX"]
    # print(x_pos)
    if x_pos > 0.08:
        re_park()
    else:
        exit()

def park_start(distance, dx, dy, yaw, ar_bool):
    global pub, motor_control
    
    steer = 7
    speed = 20
    
    if 0.7 < distance <= 1.7:
        steer = math.atan2(dx, dy)
        motor_control.angle = math.degrees(steer) * 1.5
        motor_control.speed = speed

    elif 0.1 < distance <= 0.7:
        steer_cal = yaw
        car_yaw = math.atan2(dx, dy) - yaw
        sec = time.time()
        while time.time() - sec < 3:
            motor_control.angle = -(steer - math.degrees(steer_cal)) * 1.5
            motor_control.speed = speed
            pub.publish(motor_control)
            ar_bool = arData["bool"]

        sec = time.time()
        while time.time() - sec < 2.4:
            motor_control.angle = 50
            motor_control.speed = -speed
            pub.publish(motor_control)

        while car_yaw > 0.02 :
            (_, pitch, _)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))
            car_yaw = math.atan2(arData["DX"], arData["DZ"]) - pitch

            motor_control.angle = -50
            motor_control.speed = -speed
            pub.publish(motor_control)

        parking_end(distance)

    else:
        motor_control.angle = steer
        motor_control.speed = speed

    pub.publish(motor_control)

def main():
    global pub, motor_control
    global distance

    motor_control = xycar_motor()
    
    rospy.init_node('ar_drive_info')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback, queue_size=1)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    while not rospy.is_shutdown():
        distance = math.sqrt(math.pow(arData["DX"], 2) + math.pow(arData["DZ"], 2))
        (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"], arData["AZ"], arData["AW"]))

        #print("distance: {}".format(distance))
        #print("dx: {}, dy: {}, yaw: {}".format(arData["DX"], arData["DZ"], pitch))
        park_start(distance, arData["DX"], arData["DZ"], pitch, arData["bool"])

if __name__ == "__main__":
    main()
