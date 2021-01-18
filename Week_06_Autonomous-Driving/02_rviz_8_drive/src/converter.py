#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import JointState
from xycar_motor.msg import xycar_motor

def callback(motor_pub):
    global angle, rotation, speed
    # xycar_motor 로부터 수신한 메세지 변환
    # 수신한 degree를 radian으로 변환
    angle = (motor_pub.angle * 3.14)/180
    # motor에 적용될 speed를 simulation에 맞게 바퀴의 회전 각도로 scale 변경
    speed = motor_pub.speed * 0.01
    # 메세지 발행을 위한 함수 호출
    talker(angle, rotation, speed)
    # 바퀴의 회전 각도 갱신
    rotation += speed

def talker(angle, rotation, speed):
    # JointState 메세지 타입에 내용 추가
    msg = JointState()
    msg.name = ['front_right_hinge_joint', 'front_left_hinge_joint',\
            'front_right_wheel_joint', 'front_left_wheel_joint',\
            'rear_right_wheel_joint', 'rear_left_wheel_joint']
    msg.velocity = [speed/0.01]
    msg.effort = []
    msg.header.stamp = rospy.Time.now()
    msg.position = [angle, angle, rotation, rotation, rotation, rotation]
    # 50 Hz 주기로 발송
    rate = rospy.Rate(50)
    # 노드 동기화
    while pub.get_num_connections() == 0:
        pass
    # 토픽 발행
    pub.publish(msg)
    rate.sleep()

def start():
    global pub, rotation
    
    # 바퀴의 회전 각도 초기값 설정
    rotation = 0
    # convert 노드 생성
    rospy.init_node('convert')
    # xycar_motor 토픽 구독
    rospy.Subscriber('xycar_motor', xycar_motor, callback, queue_size = 1)
    # joint_state 토픽 발행
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    start()
