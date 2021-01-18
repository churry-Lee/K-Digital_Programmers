#!/usr/bin/env python

import rospy
import time
from xycar_motor.msg import xycar_motor

motor_control = xycar_motor()
# auto_driver 노드 생성
rospy.init_node('auto_driver')
# xycar_motor 토픽 발행자 생성
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

# 토픽 발행 함수
def motor_pub(angle, speed):
    global pub
    global motor_control
    # 반환된 값을 토픽 메세지에 추가
    motor_control.angle = angle
    motor_control.speed = speed
    # 메세지 발행
    pub.publish(motor_control)

rate = rospy.Rate(50)
# 노드 동기화
while (pub.get_num_connections() == 0):
    # motor 속도 변수 설정
    speed = 3

# 초기 출발 설정
# time 모듈로 일정 시간동안만 토픽 발행하도록 설정
start = time.time()
while (time.time() - start) < 0.5:
    angle = 0.0
    motor_pub(angle, speed)
    rate.sleep()

while not rospy.is_shutdown():
    # 17.0초 동안 반시계 방향 회전
    start = time.time()
    while (time.time() - start) < 17.0:
        angle = 20
        motor_pub(angle, speed)
        rate.sleep()
    # 1.0초 동안 직진
    start = time.time()
    while (time.time() - start) < 1.0:
        angle = 0.0
        motor_pub(angle, speed)
        rate.sleep()
    # 17.0초 동안 시계방향 회전
    start = time.time()
    while (time.time() - start) < 17.0:
        angle = -20
        motor_pub(angle, speed)
        rate.sleep()
    # 1.0초 동안 직진
    start = time.time()
    while (time.time() - start) < 1.0:
        angle = 0.0
        motor_pub(angle, speed)
        rate.sleep()
