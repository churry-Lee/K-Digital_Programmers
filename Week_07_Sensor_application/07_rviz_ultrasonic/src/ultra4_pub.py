#!/usr/bin/env python

import serial, time, rospy, re
from std_msgs.msg import Int32MultiArray

ser_front = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    )

def read_sensor():

    ultrasonic_data = []

    serial_data = ser_front.readline()
    ser_front.flushInput()
    ser_front.flushOutput()
    
    serial_data = serial_data.split('mm ')
    for i, v in enumerate(serial_data):
        if i == 3:
            ultrasonic_data.append(int(v[:-4]))
        else:
            ultrasonic_data.append(int(v))
    msg.data = ultrasonic_data
  
if __name__ == '__main__':
    
    rospy.init_node('ultra4_pub', anonymous=False) # initialize node
    pub = rospy.Publisher('ultra4', Int32MultiArray, queue_size=1)

    msg = Int32MultiArray() # message type
    while not rospy.is_shutdown():
        read_sensor()
        pub.publish(msg) # publish a message
        time.sleep(0.5)
    
    ser_front.close()
