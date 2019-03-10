#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import String

left_img_path = '/media/dexter/Xubuntu/catkin_ws/src/stereo_img_publisher/img/left/000199_10.png'
right_img_path = '/media/dexter/Xubuntu/catkin_ws/src/stereo_img_publisher/img/right/000199_10.png'

cv_bridge = CvBridge()


def publish_image(left_img_publisher):
    cv_img = cv.imread(left_img_path)
    ros_img_message = cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8')
    left_img_publisher.publish(ros_img_message)
    rospy.loginfo('The image has been published')


def setup_node():
    left_img_publisher = rospy.Publisher('stereo_depth_estimation/img/left', Image, queue_size = 1)
    str_publisher = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('stereo_img_publisher', anonymous = True)
    rospy.loginfo("Starting stereo_img_publisher.")

    return left_img_publisher, str_publisher

if __name__ == "__main__":
    left_img_publisher, str_publisher = setup_node()  
    publish_image(left_img_publisher)
    str_publisher.publish('Elo')