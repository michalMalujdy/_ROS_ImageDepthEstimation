#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


def img_subscriber(value):
    rospy.loginfo('Received img {}'.format(value))


def string_sub(value):
    rospy.loginfo(value)


def main():
    rospy.init_node('stereo_depth_estimation', anonymous = True)
    rospy.Subscriber("stereo_depth_estimation/img/left", Image, img_subscriber)
    rospy.Subscriber("stereo_depth_estimation/img/right", Image, img_subscriber)
    rospy.Subscriber("chatter", String, string_sub)
    rospy.spin()


if __name__ == '__main__':
    main()