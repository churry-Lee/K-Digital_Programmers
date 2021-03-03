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
#from std_msgs.msg import Int32MultiArray

from bezier_path import *
import matplotlib.pyplot as plt
import sys

from xycar_motor.msg import xycar_motor

# arData 값 초기화
arData = {"DX":0.0, "DY":0.0, "DZ":0.0, "AX":0.0, "AY":0.0, "AZ":0.0, "AW":0.0}
roll, pitch, yaw = 0, 0, 0

target_speed = 5
k = 0.5  # control gain
kp = 0.01  # speed proportional gain
dt = 1.0  # [s] time difference
l = 2.2  # [m] wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

gx, gy, gyaw = None, None, None

class state(object):
    """
    class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """instantiate the object."""
        super(state, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        update the state of the vehicle.

        stanley control uses bicycle model.

        :param acceleration: (float) acceleration
        :param delta: (float) steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / l * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    stanley steering control.

    :param state: (state object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    compute index in the trajectory list of the target.

    :param state: (state object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # calc front axle position
    fx = state.x + l * np.cos(state.yaw)
    fy = state.y + l * np.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # project rms error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def callback(msg):
    global arData, distance

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

        distance = math.sqrt(math.pow(arData["DX"], 2)
                             + math.pow(arData["DY"], 2))


def path_planning(goal_x, goal_y, yaw):
    global state, dt

    start_x = 0.0  # [m]
    start_y = -0.0  # [m]
    start_yaw = np.radians(90.0)  # [rad]

    #end_x = goal_x  # [m]
    #end_y = goal_y  # [m]
    #yaw = yaw
    end_x = 534.0
    end_y = 642.0
    yaw = -0.05
    if end_x > 0:
        end_yaw = math.atan(end_y/end_x) - yaw  # [rad]
    elif end_x < 0:
        end_yaw = abs(math.atan(end_x/end_y)) - yaw + math.pi/2
    offset = 1.0

    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    paths_x = []
    paths_y = []
    paths_yaw = []

    for point in path:
        paths_x.append(point[0])
        paths_y.append(point[1])
        paths_yaw.append(math.atan2(point[1], point[0]))

    state = state(start_x, start_y, start_yaw, 0.0)
    last_idx = len(paths_x) - 1
    target_idx, _ = calc_target_index(state, paths_x, paths_y)

    return paths_x, paths_y, paths_yaw, target_idx


def start():
    global state, dt, distance

    motor_control = xycar_motor()

    rospy.init_node('ar_drive_info')
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, callback)
    motor_pub = rospy.Publisher('/xycar_motor', xycar_motor, queue_size =1 )

    path_x, path_y = None, None

    #input("Select Position(Input Num. 1, 2, 3 or 4 on the Simulator Window and console): ")

    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # x, y 좌표를 가지고 AR tag까지의 거리 계산(피타고라스)
        #distance = math.sqrt(math.pow(arData["DX"], 2)
        #                     + math.pow(arData["DY"], 2))
        #if distance <= 75:
        #    sys.exit()
        #    break
        if arData["DX"] != 0.0 and path_x is None:
            print("======== Start ========")
            (roll,pitch,yaw) = euler_from_quaternion((arData["AX"], arData["AY"],
                                                      arData["AZ"], arData["AW"]))

            path_x, path_y, path_yaw, target_idx = path_planning(arData["DX"],
                                                                 arData["DZ"], yaw)
            print(" DX: {}, DY: {}, Yaw: {}".format(round(arData["DX"], 0),
                                                    round(arData["DZ"], 0),
                                                    round(yaw, 5)))
            #plt.plot(path_x, path_y)
            #plt.show()

        if path_x is not None:

            # AR tag의 위치/자세 정보Quaternion 값을 euler 값으로 변환
            (roll,pitch,yaw)=euler_from_quaternion((arData["AX"], arData["AY"],
                                                    arData["AZ"], arData["AW"]))
            # radian 값을 degree로 변환
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

            speed_error = target_speed - state.v
            ai = pid_control(target_speed, state.v)
            angle, target_idx = stanley_control(state, path_x, path_y,
                                                path_yaw, target_idx)
            angle = math.degrees(angle)
            if angle >= 30.0:
                angle = 30.0
            elif angle <= -30.0:
                angle = -30.0

            print("=======================")
            print(" roll  : " + str(round(roll, 1)))
            print(" pitch : " + str(round(pitch, 1)))
            print(" yaw   : " + str(round(yaw, 1)))
            print(" x : " + str(round(state.x, 2)))
            print(" y : " + str(round(state.y, 2)))
            print(" z : " + str(round(arData["DZ"], 0)))
            print("=======================")
            print(" Distance : " + str(round(distance, 0)))
            print(" Speed: {}, Angle: {}".format(round(speed_error, 2), round(-angle, 2)))
            print("=======================")

            motor_control.angle = -angle
            motor_control.speed = speed_error
            motor_pub.publish(motor_control)

            state.update(ai, angle)
            rate.sleep()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    start()
