#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from network import ImageDepthNeuralNetwork
from cv_bridge import CvBridge
import cv2 as cv


class StereoDepthEstimationNode:
    def __init__(self):
        rospy.init_node("stereo_depth_estimation", anonymous = True)
        rospy.loginfo("Starting stereo_depth_estimation")

        network_data_dir = self.resolve_network_data_dir()
        self.depth_network = ImageDepthNeuralNetwork(network_data_dir)

        rospy.Subscriber("stereo_depth_estimation/img/left", Image, self.left_img_subscriber)
        rospy.Subscriber("stereo_depth_estimation/img/right", Image, self.right_img_subscriber)

        self.left_img_present = False
        self.right_img_present = False

        self.left_img = None
        self.right_img = None
        
        self.cv_bridge = CvBridge()

        rospy.loginfo("The node is ready")


    def resolve_network_data_dir(self):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('stereo_depth_estimation')
        return '{}/src/data'.format(root_dir)


    def left_img_subscriber(self, img):
        rospy.loginfo('Received left image')

        if self.left_img_present == False:

            self.left_img = img
            self.left_img_present = True

            if(self.both_images_present):
                self.run_network()


    def right_img_subscriber(self, img):
        rospy.loginfo('Received right image')

        if self.right_img_present == False:

            self.right_img = img
            self.right_img_present = True

            if(self.both_images_present):
                self.run_network()


    def both_images_present(self):
        return self.left_img_present and self.right_img_present

        
    def run_network(self):
        if self.left_img_present and self.right_img_present:
            cv_image_left, cv_image_right = self.convert_images_to_cv2()

            rospy.loginfo('Both side images are received, starting depth estimation...')
            depth = self.depth_network.estimate_depth(cv_image_left, cv_image_right)
            rospy.loginfo('Depth estimation finished!')

            self.left_img_present = False
            self.right_img_present = False

            cv.imshow("out", depth)
            cv.waitKey(-1)


    def convert_images_to_cv2(self):
        cv_image_left = self.cv_bridge.imgmsg_to_cv2(self.left_img, "bgr8")
        cv_image_right = self.cv_bridge.imgmsg_to_cv2(self.right_img, "bgr8")

        return cv_image_left, cv_image_right


if __name__ == "__main__":
    stereo_depth_estimation = StereoDepthEstimationNode()
    rospy.spin()