#!/usr/bin/env python

import rospy, time, tf
from math import sin, cos, pi
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

def joint_callback(msg):
    global joint

    angle = msg.position[0]
    speed = msg.velocity[0]
    
    vx = 0.07 * speed
    vy = 0.0
    vth = angle

    joint = [vx, vy, vth]

def imu_callback(data):
    global imudata

    imudata = [data.orientation.x, data.orientation.y,\
            data.orientation.z, data.orientation.w ]


def odom(joint, imudata):
    global x, y, th
    global last_time

    current_time = rospy.Time.now()

    vx, vy, vth = joint[0], joint[1], joint[2]
    
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    
    x += delta_x
    y += delta_y
    th += delta_th
    
    odom_broadcaster.sendTransform(
            (x, y, 0),
            imudata,
            current_time,
            "base_link",
            "odom"
            )
    
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*imudata))
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    
    odom_pub.publish(odom)
    
    last_time = current_time
    rate.sleep()    

def start():
    global odom_pub, odom_broadcaster
    global x, y, th
    global last_time, rate
    
    x = 0.0
    y = 0.0
    th = 0.0

    rospy.init_node('odom')
    rospy.Subscriber('joint_states', JointState, joint_callback, queue_size=1)
    rospy.Subscriber('imu', Imu, imu_callback, queue_size=1)

    odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    while odom_pub.get_num_connections() == 0:
        last_time = rospy.Time.now()
    
    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        odom(joint, imudata)

if __name__ == '__main__':
    start()
