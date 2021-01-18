#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import time

import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

def callback(msg):
    global vx, vy, vth, last_time, rate
    # joint_state 메세지로 부터 받은 데이터 중 필요한 정보 빼옴
    angle = msg.position[0]
    speed = msg.velocity[0]
    # 선속도 및 각속도 설정
    vx = 0.07 * speed
    vy = 0.0
    vth = angle
    # 노드 동기화
    while odom_pub.get_num_connections() == 0:
        # 메세지 발행에 필요한 시간 정보 변수 설정
        last_time = rospy.Time.now()

    rate = rospy.Rate(50)
    # odom 함수를 호출하여 발행 준비, 선속도, 각속도를 인자로 사용
    odom(vx, vy, vth)


def odom(vx, vy, vth):
    global x, y, th, last_time

    current_time = rospy.Time.now()
    # 속도 변화량 계산, 시간을 곱해서 이동거리 변화량으로 나타냄
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    # 이동거리 갱신
    x += delta_x
    y += delta_y
    th += delta_th
    # euler angle을 quaternion으로 변환
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    # base_link와 odom을 연결
    odom_broadcaster.sendTransform(
            (x, y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
            )
    # Odometry 메세지 내용 채우기
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = 'odom'
    # 위치 정보 채워 넣기
    odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
    odom.child_frame_id = "base_link"
    # 속도 정보 채워넣기
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
    # odometry 정보 토픽 발행
    odom_pub.publish(odom)
    # 시간 정보 갱신
    last_time = current_time
    rate.sleep()
    

def start():
    global odom_pub, odom_broadcaster, x, y, th
    # xycar의 초기 위치 변수 설정    
    x = 0.0
    y = 0.0
    th = 0.0
    # odometry_publisher 노드 생성
    rospy.init_node('odometry_publisher')
    # joint_states 토픽 구독
    rospy.Subscriber('joint_states', JointState, callback, queue_size=1)
    # odom 토픽 발행자 준비
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
    # odometry에서는 토픽 발행자 뿐만 아니라, tf도 같이 발행해주어야함
    odom_broadcaster = tf.TransformBroadcaster()
    rospy.spin()

if __name__ == '__main__':
    start()
