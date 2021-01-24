#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_motor.msg import xycar_motor

distance = []
motor_msg = xycar_motor()

def callback(data):
    global distance

    distance = data.data

def drive_go():
    global motor_msg, pub
    
    motor_msg.speed = 20
    motor_msg.angle = 0
    pub.publish(motor_msg)

def drive_stop(angle):
    global motor_msg, pub
    
    motor_msg.speed = 0
    motor_msg.angle = 0 
    pub.publish(motor_msg)
 
def drive_back(angle, speed):
    global motor_msg, pub
    
    motor_msg.speed = speed
    motor_msg.angle = angle 
    pub.publish(motor_msg)

rospy.init_node('lidar_driver')
rospy.Subscriber("/scan", LaserScan, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(3) #ready to connect lidar 

while not rospy.is_shutdown():
    ok = 0
    for degree in range(60, 120):
        if distance[degree] <= 0.3:
            ok += 1
        if ok > 3:
            drive_stop()
            time.sleep(0.5)
            drive_back(50, -20)
            time.sleep(1)
            break
            
    if ok <= 3:
        drive_go()
