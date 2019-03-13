#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from std_msgs.msg import String
import sys

class StereoImagePublisherNode:
    def __init__(self, set_name):
        self.set_name = set_name
        self.cv_bridge = CvBridge()

        self.resolve_images_path()
        
        self.setup_node()

    
    def resolve_images_path(self):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('stereo_img_publisher')
        self.left_img_path = '{}/img/left/{}.png'.format(root_dir, self.set_name)
        self.right_img_path = '{}/img/right/{}.png'.format(root_dir, self.set_name)


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


def getSetNameFromArgs():
    set_name = '1'

    if(len(sys.argv) > 1 and '-set2' in sys.argv):
        set_name = '2'

    
    if(len(sys.argv) > 1 and '-set3' in sys.argv):
        set_name = '3'

    return set_name

if __name__ == "__main__":
    ros_node = StereoImagePublisherNode(getSetNameFromArgs())
    ros_node.publish_left_img()
    ros_node.publish_right_img()