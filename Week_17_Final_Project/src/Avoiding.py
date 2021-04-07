#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math, time
import numpy as np

from sensor_msgs.msg import LaserScan

obstacle = False
distance = [0 for _ in range(505)]

def obstacle_cnt(distance, dist_lidar, range):
    lidar_count_l = 0
    lidar_count_r = 0
    lidar = distance
    for r_point in lidar[250 - range:250]:
        if r_point != 0.0 and r_point < dist_lidar:
            lidar_count_r += 1
    for l_point in lidar[250:250 + range + 1]:
        if l_point != 0.0 and l_point < dist_lidar:
            lidar_count_l += 1
    lidar_obs_count = lidar_count_l + lidar_count_r

    return [lidar_obs_count, lidar_count_l, lidar_count_r]

def avoid_decision(total, left, right):
    speed = 30

    if left+3 < right:
        steer = -40
    elif left > right+3:
        steer = 40
    else:
        steer = 7

    return steer, speed

def lidar_callback(data):
    global distance
    
    distance = data.ranges

def main():
    global obstacle, distance

    rospy.init_node('lidar_data')
    rospy.Subscriber("/scan", LaserScan, lidar_callback, queue_size=1)

    while not rospy.is_shutdown():
        [lidar_obs_count, lidar_count_l, lidar_count_r] = obstacle_cnt(distance)
        delta, v = avoid_decision(lidar_obs_count, lidar_count_l, lidar_count_r)
        # print("Total: {}, left_obs: {}, right_obs: {}".format(lidar_obs_count, lidar_count_l, lidar_count_r))

if __name__ == "__main__":
    main()