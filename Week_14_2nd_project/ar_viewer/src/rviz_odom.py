#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState

global Angle, current_speed

current_speed = 0

def callback(msg):  
       global Angle, current_speed

       current_speed = msg.velocity[0] * 0.1
       Angle = msg.position[msg.name.index("front_left_hinge_joint")]

rospy.Subscriber('/joint_states', JointState, callback, queue_size=1)
rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(30.0)

#current_speed = 0.4

wheel_base = 0.2
x_ = 0
y_ = 0
yaw_ = 0

Angle = 0
while not rospy.is_shutdown():

    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()

    current_steering_angle = Angle
    current_angular_velocity = current_speed * math.tan(current_steering_angle) / wheel_base

    x_dot = current_speed * cos(yaw_)
    y_dot = current_speed * sin(yaw_)   
    x_ += x_dot * dt;
    y_ += y_dot * dt;
    yaw_ += current_angular_velocity * dt 

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, yaw_)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x_, y_, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x_, y_, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    #odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, yaw_))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
