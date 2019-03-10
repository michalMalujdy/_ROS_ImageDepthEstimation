#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import String

class StereoImagePublisherNode:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.left_img_path = '/media/dexter/Xubuntu/catkin_ws/src/stereo_img_publisher/img/left/000199_10.png'
        self.right_img_path = '/media/dexter/Xubuntu/catkin_ws/src/stereo_img_publisher/img/right/000199_10.png'
        
        self.setup_node()

    
    def setup_node(self):
        self.left_img_publisher = rospy.Publisher('stereo_depth_estimation/img/left', Image, queue_size = 1)
        rospy.init_node('stereo_img_publisher', anonymous = True)
        rospy.loginfo("Starting stereo_img_publisher.")


    def publish_image(self):
        cv_img = cv.imread(self.left_img_path)
        ros_img_message = self.cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        self.left_img_publisher.publish(ros_img_message)
        rospy.loginfo('The image has been published')


if __name__ == "__main__":
    ros_node = StereoImagePublisherNode()
    ros_node.publish_image()