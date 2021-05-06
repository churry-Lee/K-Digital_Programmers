#!/usr/bin/env python

import rospy
import time
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

motor_control = xycar_motor()
distance = [0,0,0,0,0,0,0,0]
rospy.init_node('auto_driver')

pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def callback(data):
    global distance 
    distance = data.data

def drive_go(speed,angle):
    motor_control.angle = angle
    motor_control.speed = speed 
    pub.publish(motor_control)

def drive_stop(speed,angle):    
    motor_control.speed = 0
    motor_control.angle = 0
    pub.publish(motor_control)
    time.sleep(0.1)
    for i in range(1,10):
        motor_control.speed = -i*3
        motor_control.angle = 0
        pub.publish(motor_control)
        time.sleep(0.05)
    motor_control.speed = -30
    motor_control.angle = 0
    pub.publish(motor_control)
    time.sleep(0.5)
    drive_go(speed,angle)
    time.sleep(0.5)

sub = rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, callback, queue_size = 1)

while not rospy.is_shutdown():

    if distance[2] < 20:
        if int(distance[1]) > int(distance[3]):
            drive_stop(30,-50)
        else:
            drive_stop(30,50)        
    else :
        if abs(int(distance[1]) - int(distance[3]))<10:
            drive_go(30,0)
        elif int(distance[1]>distance[3]):
            drive_go(25,-20)
        else:
            drive_go(25,20)
    time.sleep(0.1)
