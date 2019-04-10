#!/usr/bin/env python

import rospy
import rospkg
from sensor_msgs.msg import Image
from network import ImageDepthNeuralNetwork
from cv_bridge import CvBridge
import cv2 as cv
import sys


class StereoDepthEstimationNode:
    def __init__(self, showResultDepthImage = False):
        self.setup_initial_properties()

        rospy.init_node("stereo_depth_estimation")
        rospy.loginfo("Starting stereo_depth_estimation")
        network_data_dir = self.resolve_network_data_dir()
        self.depth_network = ImageDepthNeuralNetwork(network_data_dir)

        self.setup_input_topics_subscriptions()
        self.setup_output_publisher(self.img_out_topic_name, self.default_img_out_topic)
        
        self.print_config_info()

    def setup_initial_properties(self):
        self.node_name = 'stereo_depth_estimation'

        self.default_img_topic_left = '/stereo_depth_estimation/input/left'
        self.default_img_topic_right = '/stereo_depth_estimation/input/right'

        self.img_topic_left_name = '/stereo_depth_estimation/input_img_left'
        self.img_topic_right_name = '/stereo_depth_estimation/input_img_right'

        self.img_out_topic_name = '/stereo_depth_estimation/img_out'
        self.default_img_out_topic = '/stereo_depth_estimation/output'

        self.left_img_present = False
        self.right_img_present = False

        self.left_img = None
        self.right_img = None

        self.cv_bridge = CvBridge()

        self.showResultDepthImage = showResultDepthImage


    def resolve_network_data_dir(self):
        rospack = rospkg.RosPack()
        root_dir = rospack.get_path('stereo_depth_estimation')
        return '{}/src/data'.format(root_dir)


    def setup_input_topics_subscriptions(self):
        self.left_img_topic, self.right_img_topic = self.resolve_input_topics(
            self.default_img_topic_left, 
            self.default_img_topic_right)

        rospy.Subscriber(self.left_img_topic, Image, self.left_img_subscriber)
        rospy.Subscriber(self.right_img_topic, Image, self.right_img_subscriber)


    def resolve_input_topics(
        self, 
        default_img_topic_left, 
        default_img_topic_right):

        left_topic = rospy.get_param('/stereo_depth_estimation/input_img_left', default_img_topic_left)
        right_topic = rospy.get_param('/stereo_depth_estimation/input_img_right', default_img_topic_right)

        return left_topic, right_topic

    def setup_output_publisher(self, img_out_topic_name, default_img_out_topic):
        if rospy.has_param(img_out_topic_name):
            self.img_out_topic = rospy.get_param(img_out_topic_name)
        else:
            self.img_out_topic = default_img_out_topic
            rospy.set_param(img_out_topic_name, default_img_out_topic)

        self.img_out_topic = rospy.get_param(img_out_topic_name, default_img_out_topic)
        self.result_publisher = rospy.Publisher(self.img_out_topic, Image, queue_size = 5)


    def left_img_subscriber(self, img):
        if self.left_img_present == False:
            rospy.loginfo('Received left image')

            self.left_img = img
            self.left_img_present = True

            if(self.both_images_present):
                self.run_network()


    def right_img_subscriber(self, img):
        if self.right_img_present == False:
            rospy.loginfo('Received right image')

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

            ros_img_message = self.cv_bridge.cv2_to_imgmsg(depth, '16UC1')
            self.result_publisher.publish(ros_img_message)

            self.left_img_present = False
            self.right_img_present = False

            if(self.showResultDepthImage):
                cv.imshow("out", depth)
                cv.waitKey(-1)


    def convert_images_to_cv2(self):
        cv_image_left = self.cv_bridge.imgmsg_to_cv2(self.left_img, "rgb8")
        cv_image_right = self.cv_bridge.imgmsg_to_cv2(self.right_img, "rgb8")

        return cv_image_left, cv_image_right


    def print_config_info(self):
        print('----------------------------------------------------')
        print("The node is ready")
        print('Input left image topic: {}'.format(self.left_img_topic))
        print('Input right image topic: {}'.format(self.right_img_topic))
        print('Output depth image topic: {}'.format(self.img_out_topic))
        print('----------------------------------------------------')


if __name__ == "__main__":
    showResultDepthImage = len(sys.argv) > 1 and '-show' in sys.argv

    stereo_depth_estimation = StereoDepthEstimationNode(showResultDepthImage)

    rospy.spin()