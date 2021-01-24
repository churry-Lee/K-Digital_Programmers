#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from xycar_motor.msg import xycar_motor

ultra_msg = None
motor_msg = xycar_motor()

def callback(data):
    global front, l_front, r_front
    ultra_msg = data.data

    front = ultra_msg[2]
    l_front = ultra_msg[3]
    r_front = ultra_msg[1]

def drive_go():
    global motor_msg, pub

    motor_msg.speed = 20
    motor_msg.angle = 0
    pub.publish(motor_msg)

def drive_stop(angle):
    global motor_msg, pub
    
    motor_msg.speed = 0
    motor_msg.angle = angle 
    pub.publish(motor_msg)
 
def drive_back(angle, speed):
    global motor_msg, pub
    
    motor_msg.speed = speed
    motor_msg.angle = angle 
    pub.publish(motor_msg)

rospy.init_node('ultra_driver')
rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, callback, queue_size=1)
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

time.sleep(2) #ready to connect ultra 

angle = 0

while not rospy.is_shutdown():
    if (front > 0 and front < 20) or (l_front > 0 and l_front < 15)\
                 or (r_front > 0 and r_front < 15):
        drive_stop(angle)
        time.sleep(0.5)
	
        if l_front > r_front:
	    angle = 50
            speed = -20
	    while True:
            	drive_back(angle, speed)
		if l_front > 0 and l_front > 15:
		    break

        elif l_front < r_front:
            angle = -50
            speed = -20
	    while True:
	        drive_back(angle, speed)
        	if r_front > 0 and r_front > 15:
		    break
 	
	time.sleep(1)
	drive_go()

    else:
        drive_go()


