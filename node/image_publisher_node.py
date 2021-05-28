#!/home/wyf/anaconda3/envs/test/bin/python
#!coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys

import test.test
 
def imagePublisher():
    rospy.init_node('image_publisher', anonymous=True)

    img_pub = rospy.Publisher('/image_source', Image, queue_size=10)
    rate = rospy.Rate(25)
 
    cap = cv2.VideoCapture("/home/wyf/ros_ws/src/TrafficCount/video/1_1080.mp4")

    bridge = CvBridge()
 
    if not cap.isOpened():
        print('Video open failed!')
        return -1
 
    count = 0
    # scaling_factor =0.5
    print('Video open succeed!')
    while not rospy.is_shutdown():
        if count >= 4500:
            cap = cv2.VideoCapture("/home/wyf/ros_ws/src/TrafficCount/video/1_1080.mp4")
            count = 0
        ret, frame = cap.read()
        # frame = cv2.resize(frame,None,fx=scaling_factor,fy=scaling_factor,interpolation=cv2.INTER_AREA)
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_pub.publish(msg)
        count = count + 1
        print('publish', count, 'frame')
        rate.sleep()
 
if __name__ == '__main__':
    try:
        imagePublisher()
    except rospy.ROSInterruptException:
        pass
