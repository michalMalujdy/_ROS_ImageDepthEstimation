#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import String

class StereoImagePublisherNode:
    def __init__(self):
        self.cv_bridge = CvBridge()

        self.resolve_images_path()
        
        self.setup_node()

    
    def resolve_images_path(self):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('stereo_img_publisher')
        self.left_img_path = '{}/img/left/000199_10.png'.format(root_dir)
        self.right_img_path = '{}/img/right/000199_10.png'.format(root_dir)


    def setup_node(self):
        self.left_img_publisher = rospy.Publisher('stereo_depth_estimation/img/left', Image, queue_size = 1)
        self.right_img_publisher = rospy.Publisher('stereo_depth_estimation/img/right', Image, queue_size = 1)
        rospy.init_node('stereo_img_publisher', anonymous = True)
        rospy.loginfo("Starting stereo_img_publisher.")


    def publish_left_img(self):
        cv_img = cv.imread(self.left_img_path)
        ros_img_message = self.cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        self.left_img_publisher.publish(ros_img_message)
        rospy.loginfo('The left image has been published')


    def publish_right_img(self):
        cv_img = cv.imread(self.right_img_path)
        ros_img_message = self.cv_bridge.cv2_to_imgmsg(cv_img, 'bgr8')
        self.right_img_publisher.publish(ros_img_message)
        rospy.loginfo('The right image has been published')


if __name__ == "__main__":
    ros_node = StereoImagePublisherNode()
    ros_node.publish_left_img()
    ros_node.publish_right_img()