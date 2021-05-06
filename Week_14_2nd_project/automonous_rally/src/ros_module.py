#!/usr/bin/env python

import rospy, cv2
import numpy as np
from xycar_motor.msg import xycar_motor
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Imu, Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from ar_track_alvar_msgs.msg import AlvarMarkers

class ROS:

    bridge = CvBridge()
    motor_msg = xycar_motor()
    ultrasonic_data = {"FL":0, "FM":0, "FR":0, "L":0, "BL":0, "BM":0, "BR":0, "R":0}
    imu_data = {"x":0.0, "y":0.0, "z":0.0, "w":0.0}
    cam_image = np.empty(shape=[0])
    darknet_detect_cnt = 0
    darknet_image = np.empty(shape=[0])
    bounding_boxes = []
    ar_tags = []

    mtx = np.array([[422.037858, 0.0, 245.895397], [0.0, 435.589734, 163.625535], [0.0, 0.0, 1.0]])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (640, 480), 1, (640, 480))

    def __init__(self, name, cal=True):
        rospy.init_node(name)
        self.calibration = cal
        self.motor_pub = rospy.Publisher("xycar_motor", xycar_motor, queue_size=1)
        rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, self.ultrasonic_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        rospy.Subscriber("usb_cam/image_raw", Image, self.camera_callback)
        rospy.Subscriber("/darknet_ros/found_object", ObjectCount, self.darknet_ros_detect_object_cnt)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.darknet_bounding_box_data)
        rospy.Subscriber("/darknet_ros/detection_image", Image, self.darknet_ros_image_callback)
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.ar_tags_data)

    def get_ultrasonic_data(self):
        return self.ultrasonic_data
    
    def get_imu_data(self):
        return self.imu_data

    def get_camera_image_data(self):
        return self.cam_image
    
    def get_darknet_detect_count_data(self):
        return self.darknet_detect_cnt

    def get_darknet_image_data(self):
        return self.darknet_image

    def get_darknet_bounding_boxes(self):
        return self.bounding_boxes

    def get_ar_tags_datas(self):
        return self.ar_tags

    def set_motor(self, angle, speed):
        self.motor_msg.header.stamp = rospy.Time.now()
        self.motor_msg.angle = angle
        self.motor_msg.speed = speed
        self.motor_pub.publish(self.motor_msg)

    def ultrasonic_callback(self, data):
        self.ultrasonic_data["L"] = data.data[0]
        self.ultrasonic_data["FL"] = data.data[1]
        self.ultrasonic_data["FM"] = data.data[2]
        self.ultrasonic_data["FR"] = data.data[3]
        self.ultrasonic_data["R"] = data.data[4]
        self.ultrasonic_data["BR"] = data.data[5]
        self.ultrasonic_data["BM"] = data.data[6]
        self.ultrasonic_data["BL"] = data.data[7]

    def imu_callback(self, data):
        self.imu_data["x"] = data.orientation.x
        self.imu_data["y"] = data.orientation.y
        self.imu_data["z"] = data.orientation.z
        self.imu_data["w"] = data.orientation.w

    def calibrate_image(self, image):
        tf_image = cv2.undistort(image, self.mtx, self.dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        tf_image = tf_image[y:y+h, x:x+w]

        return cv2.resize(tf_image, (640, 480))

    def camera_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        if self.calibration:
            self.cam_image = self.calibrate_image(image)
        else:
            self.cam_image = image

    def darknet_ros_detect_object_cnt(self, data):
        self.darknet_detect_cnt = data.count

    def darknet_ros_image_callback(self, data):
        self.darknet_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def darknet_bounding_box_data(self, data):
        d = []
        b = {"probability":0.0, "xmin":0, "ymin":0, "xmax":0, "ymax":0, "id":0, "class":""}
        for bb in data.bounding_boxes:
            b["probability"] = bb.probability
            b["xmin"] = bb.xmin
            b["ymin"] = bb.ymin
            b["xmax"] = bb.xmax
            b["ymax"] = bb.ymax
            b["id"] = bb.id
            b["class"] = bb.Class
            d.append(b)
        self.bounding_boxes = d

    def ar_tags_data(self, data):
        #d = []
        b = {"id":0, "pos_x":0.0, "pos_y":0.0, "pos_z":0.0, "ori_x":0.0, "ori_y":0.0, "ori_z":0.0, "ori_w":0.0}
        for bb in data.markers:
            b["id"] = bb.id
            b["pos_x"] = bb.pose.pose.position.x
            b["pos_y"] = bb.pose.pose.position.y
            b["pos_z"] = bb.pose.pose.position.z
            b["ori_x"] = bb.pose.pose.orientation.x
            b["ori_y"] = bb.pose.pose.orientation.y
            b["ori_Z"] = bb.pose.pose.orientation.z
            b["ori_w"] = bb.pose.pose.orientation.w
            #d.append(b)
        self.ar_tags = b

    def get_shutdown(self):
        return not rospy.is_shutdown()
