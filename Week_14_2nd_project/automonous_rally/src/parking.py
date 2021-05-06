#/usr/bin/env python
#-*- coding: utf-8 -*-

import cv2, math, time
import numpy as np
from tf.transformations import euler_from_quaternion

import rospy
from xycar_motor.msg import xycar_motor


class PARK:
    angle = 0
    speed = 0
    
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_data(self, data):
        angle, speed = start(data)
        self.set_motor(angle, speed)
    
    def get_motor(self):
        return self.angle, self.speed


def go_drive(v, dx, dy, yaw):
    global park
    
    yaw = math.radians(yaw)

    if yaw == 0:
        if dx > 0:
            delta = math.atan2(dx, dy)
        elif dx < 0:
            delta = math.atan2(dx, dy)
        elif dx == 0:
            delta = 0
    elif yaw > 0:
        if dx > 0:
            delta = math.atan2(dx, dy) - yaw
        elif dx < 0:
            delta = math.atan2(dx, dy) - yaw
        elif dx == 0:
            delta = -yaw
    elif yaw < 0:
        if dx > 0:
            delta = math.atan2(dx, dy) - yaw
        elif dx < 0:
            delta = math.atan2(dx, dy) - yaw
        elif dx == 0:
            delta = -yaw
    
    delta = int(math.degrees(delta))

    return delta * 2.0

def start(arData):
    global park
    
    park = PARK()
    # x, y 좌표를 가지고 AR tag까지의 거리 계산(피타고라스)
    distance = math.sqrt(math.pow(arData["pos_x"], 2) + math.pow(arData["pos_z"], 2))
    # AR tag의 위치/자세 정보Quaternion 값을 euler 값으로 변환
    (roll,pitch,yaw)=euler_from_quaternion((arData["ori_x"], arData["ori_y"],
                                            arData["ori_z"], arData["ori_w"]))
    
    if 0 <= distance <= 0.3:
        angle, speed = 0, 0
    
    elif 0.3 < distance <= 2.0:
        speed = 40
        angle = go_drive(speed, arData["pos_x"], arData["pos_z"], pitch)
    else:
        speed = 50
        angle = go_drive(speed, arData["pos_x"], arData["pos_z"], pitch)
    
    # radian 값을 degree로 변환
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    
    print("=======================")
    #print(" roll  : " + str(round(roll, 1)))
    #print(" pitch : " + str(round(yaw, 1)))
    print(" yaw   : " + str(round(pitch, 2)))

    print(" x: " + str(round(arData["pos_x"], 2)))
    print(" y: " + str(round(arData["pos_z"], 2)))
    print(" dist: " + str(round(distance, 2)))
    #print(" z : " + str(round(arData["DY"],0)))

    return angle, speed

