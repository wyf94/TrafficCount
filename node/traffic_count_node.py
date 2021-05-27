#!/home/wyf/anaconda3/envs/test/bin/python
#!coding=utf-8

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
import cv_bridge
from cv_bridge import CvBridge, CvBridgeError

# import test.test as test
import detector.detector as detector
import tracker.tracker as tracker



def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    cv2.imshow("lala",cv_image)
    cv2.waitKey(0)

def showImage():
    rospy.init_node('showImage',anonymous = True)
    rospy.Subscriber('/image_source', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    showImage()

