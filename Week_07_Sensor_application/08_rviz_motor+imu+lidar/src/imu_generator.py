#!/usr/bin/env python

import rospy, time, os, math, tf
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler

def msg_publish(quat):
    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = 'map'
    imu.orientation.x = quat[0]
    imu.orientation.y = quat[1]
    imu.orientation.z = quat[2]
    imu.orientation.w = quat[3]
    rate = rospy.Rate(20)
    pub.publish(imu)
    rate.sleep()



def euler2quat(roll, pitch, yaw):
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    msg_publish(quat)

def data_converter(data):
    angle = data.split(" ")

    roll = float(angle[2][:-1])
    pitch = float(angle[5][:-1])
    yaw = float(angle[8][:-1])

    euler2quat(roll, pitch, yaw)

def start():
    global pub, r

    rospy.init_node('imu_generator')
    pub = rospy.Publisher('imu', Imu, queue_size=10)

    r = open("/home/vm-ubuntu/my_ws/src/rviz_all/src/imu_data.txt", "rt")

    while True:
        data = r.readline()
        if not data: break
        data_converter(data)

if __name__ == "__main__":
    try:
        start()
    finally:
        r.close()
