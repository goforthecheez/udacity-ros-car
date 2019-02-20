#!/usr/bin/env python
import time
import math
import sys
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class TLImageCollector(object):

    def __init__(self):
        """
        :param is_testing: in case class has to run in test environment without ros, and we don't want ros sub/pub to fail
        """

        self.bridge = CvBridge()

        rospy.init_node('tl_image_collector')

        rospy.Subscriber('/image_color', Image, self.image_cb)
        self.img_cnt = 0


        rospy.spin()


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.img_cnt % 9 != 0:
            self.img_cnt += 1
            return

        self.img_cnt = 1
        # check if it's time to process the next image
        # timestamp_now = rospy.get_rostime()
        # if (self.timestamp_before == None):
        #     # it's the very first image
        #     self.timestamp_before = timestamp_now
        #     self.skipping_duration = rospy.Duration(SKIPPING_DURATION)
        # else:
        #     time_elapsed = timestamp_now - self.timestamp_before
        #     if (time_elapsed < self.skipping_duration):
        #         # do not process this image, wait until enough time has elapsed
        #         return
        #     else:
        #         # set current timestamp and next duration, take jitter into account
        #         self.timestamp_before = timestamp_now
        #         self.skipping_duration += rospy.Duration(SKIPPING_DURATION) - time_elapsed

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        directory = '<path_to_save_images>'
        if not os.path.exists(directory):
            os.makedirs(directory)
        cv2.imwrite('{}/img{}.jpg'.format(directory, time.time()), cv_image)


if __name__ == '__main__':
    try:
        TLImageCollector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start image collector node.')
