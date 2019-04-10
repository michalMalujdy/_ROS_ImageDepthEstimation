#!/usr/bin/env python

import rospy 
import cv2
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class StereoImgSaverNode:
    def __init__(self, prefix = '0'):
        self.cv_bridge = CvBridge()
        self.prefix = prefix
        self.is_left_img_received = False
        self.is_right_img_received = False

        rospy.init_node("stereo_img_saver", anonymous = True)
        rospy.loginfo("Starting StereoImgSaverNode.")
        rospy.Subscriber("stereo/left/image_raw", Image, self.left_img_subscriber)
        rospy.Subscriber("stereo/right/image_raw", Image, self.right_img_subscriber)

    def left_img_subscriber(self, image):
        if not self.is_left_img_received:
            cv_img = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite('{}_left.png'.format(self.prefix), cv_img)
            self.is_left_img_received = True

    def right_img_subscriber(self, image):
        if not self.is_right_img_received:
            cv_img = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
            cv2.imwrite('{}_right.png'.format(self.prefix), cv_img)
            self.is_right_img_received = True


if __name__ == "__main__":
    parser = argparse.ArgumentParser('parser')
    parser.add_argument('-p', '--prefix')
    args = parser.parse_args()

    stereo_img_saver = StereoImgSaverNode(args.prefix)
    rospy.spin()