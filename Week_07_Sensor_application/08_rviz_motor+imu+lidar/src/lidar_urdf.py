#!/usr/bin/env python

import rospy, math, time
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import Header

def lidar_callback(data):
    lidar_point = data.ranges

    if lidar_point == None:
        pass

    f_dist, l_dist, b_dist, r_dist = lidar_point[0], lidar_point[3],\
            lidar_point[6], lidar_point[9]

    lidar_pub(f_dist, l_dist, b_dist, r_dist)

def lidar_pub(f_dist, l_dist, b_dist, r_dist):
    h.frame_id = "front"
    msg.header = h
    msg.header.stamp = rospy.Time.now()
    msg.range = f_dist
    pub1.publish(msg)
    time.sleep(0.1)

    h.frame_id = "left"
    msg.header = h
    msg.header.stamp = rospy.Time.now()
    msg.range = l_dist
    pub2.publish(msg)
    time.sleep(0.1)
    
    h.frame_id = "back"
    msg.header = h
    msg.header.stamp = rospy.Time.now()
    msg.range = b_dist
    pub3.publish(msg)
    time.sleep(0.1)
    
    h.frame_id = "right"
    msg.header = h
    msg.header.stamp = rospy.Time.now()
    msg.range = r_dist
    pub4.publish(msg)
    time.sleep(0.1)

def start():
    global pub1, pub2, pub3, pub4, lidar_point, msg, h

    lidar_point = None
    
    h = Header()
    
    msg = Range()
    msg.radiation_type = Range().ULTRASOUND
    msg.min_range = 0.02
    msg.max_range = 3.0
    msg.field_of_view = (30.0/180.0) * math.pi

    rospy.init_node('lidar')
    pub1 = rospy.Publisher('scan1', Range, queue_size=1)
    pub2 = rospy.Publisher('scan2', Range, queue_size=1)
    pub3 = rospy.Publisher('scan3', Range, queue_size=1)
    pub4 = rospy.Publisher('scan4', Range, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    rospy.spin()

if __name__ == "__main__":
    start()
