#!/usr/bin/env python

import rospy
import time
from xycar_motor.msg import xycar_motor

motor_control = xycar_motor()
rospy.init_node('auto_driver')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

def motor_pub(angle, speed):
    global pub
    global motor_control

    motor_control.angle = angle
    motor_control.speed = speed

    pub.publish(motor_control)

rate = rospy.Rate(50)
while (pub.get_num_connections() == 0):
    speed = 3

start = time.time()
while (time.time() - start) < 0.5:
    angle = 0.0
    motor_pub(angle, speed)
    rate.sleep()


while not rospy.is_shutdown():

    start = time.time()
    while (time.time() - start) < 17.0:
        angle = 20
        motor_pub(angle, speed)
        rate.sleep()
    
    start = time.time()
    while (time.time() - start) < 1.0:
        angle = 0.0
        motor_pub(angle, speed)
        rate.sleep()
    
    start = time.time()
    while (time.time() - start) < 17.0:
        angle = -20
        motor_pub(angle, speed)
        rate.sleep()

    start = time.time()
    while (time.time() - start) < 1.0:
        angle = 0.0
        motor_pub(angle, speed)
        rate.sleep()
