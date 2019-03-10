#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from network import estimate_depth


left_img = None
left_img_present = False

right_img = None
right_img_present = False

def run_network_if_both_images_are_present():
    global left_img_present, right_img_present
    if left_img_present and right_img_present:
        rospy.loginfo('Both side images are received, starting depth estimation...')
        depth = estimate_depth(left_img, right_img)
        #depth = 1 # to remove once tensorflow and network is set
        rospy.loginfo('Depth estimation finished!')
        rospy.loginfo('Depth: {}'.format(depth))

        left_img_present = False
        right_img_present = False


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