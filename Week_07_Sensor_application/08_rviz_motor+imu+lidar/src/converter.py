#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import JointState
from xycar_motor.msg import xycar_motor

def callback(motor_pub):
    global angle, rotation, speed

    angle = (motor_pub.angle * 3.14)/180
    speed = motor_pub.speed * 0.01

    start = time.time()

    talker(angle, rotation, speed)
    rotation += speed

def talker(angle, rotation, speed):
    
    msg = JointState()
    
    msg.name = ['front_right_hinge_joint', 'front_left_hinge_joint',\
            'front_right_wheel_joint', 'front_left_wheel_joint',\
            'rear_right_wheel_joint', 'rear_left_wheel_joint']
    msg.velocity = [speed/0.01]
    msg.effort = []
    msg.header.stamp = rospy.Time.now()

    rate = rospy.Rate(50)
    msg.position = [angle, angle, rotation, rotation, rotation, rotation]
    
    while pub.get_num_connections() == 0:
        pass

    pub.publish(msg)
    rate.sleep()

def start():
    global pub, rotation

    rotation = 0
    rospy.init_node('convert')
    rospy.Subscriber('xycar_motor', xycar_motor, callback, queue_size = 1)
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    start()
