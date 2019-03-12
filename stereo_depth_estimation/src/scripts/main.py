#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from network import estimate_depth
from cv_bridge import CvBridge
import cv2 as cv


left_img = None
left_img_present = False

right_img = None
right_img_present = False

cv_bridge = CvBridge()

def convert_images_to_cv2():
    cv_image_left = cv_bridge.imgmsg_to_cv2(left_img, "bgr8")
    cv_image_right = cv_bridge.imgmsg_to_cv2(right_img, "bgr8")

    return cv_image_left, cv_image_right


def run_network_if_both_images_are_present():
    global left_img_present, right_img_present
    if left_img_present and right_img_present:
        cv_image_left, cv_image_right = convert_images_to_cv2()

        rospy.loginfo('Both side images are received, starting depth estimation...')
        depth = estimate_depth(cv_image_left, cv_image_right)
        rospy.loginfo('Depth estimation finished!')

        left_img_present = False
        right_img_present = False

        cv.imshow("out", depth)
        cv.waitKey(-1)


def left_img_subscriber(img):
    global left_img, left_img_present
    rospy.loginfo('Received left image')

    if left_img_present == False:
        left_img = img
        left_img_present = True
        run_network_if_both_images_are_present()



def right_img_subscriber(img):
    global right_img, right_img_present
    rospy.loginfo('Received right image')

    if right_img_present == False:
        right_img = img
        right_img_present = True
        run_network_if_both_images_are_present()


def string_sub(value):
    rospy.loginfo(value)


def main():
    rospy.init_node('stereo_depth_estimation', anonymous = True)
    rospy.Subscriber("stereo_depth_estimation/img/left", Image, left_img_subscriber)
    rospy.Subscriber("stereo_depth_estimation/img/right", Image, right_img_subscriber)
    rospy.spin()


if __name__ == '__main__':
    main()