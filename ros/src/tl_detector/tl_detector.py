#!/usr/bin/env python
import time
import math
import sys
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 2
SKIPPING_DURATION = 0.5 # time in seconds to wait until next camera image will be processed
MAX_WP_DISTANCE_TO_CLASSIFY_TL = 100

class TLDetector(object):

    def __init__(self, is_testing=False):
        """
        :param is_testing: in case class has to run in test environment without ros, and we don't want ros sub/pub to fail
        """

        self.is_testing = is_testing

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None

        self.lights = []

        self.bridge = CvBridge()
        # self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        if not self.is_testing:
            rospy.init_node('tl_detector')

            config_string = rospy.get_param("/traffic_light_config")
            self.config = yaml.load(config_string)
            self.light_classifier = TLClassifier(self.config['is_site'])
            # try to run classifier the first time with the booster image
            # - It will increase performance for the first time classification
            booster_image = cv2.imread(r'boost_image/booster.jpg')
            self.light_classifier.get_classification(booster_image)
            
            # List of positions that correspond to the line to stop in front of
            # for a given intersection
            self.stop_line_positions = self.config['stop_line_positions']

            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

            '''
            /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
            helps you acquire an accurate ground truth data source for the traffic light
            classifier by sending the current color state of all traffic lights in the
            simulator. When testing on the vehicle, the color state will not be available. You'll need to
            rely on the position of the light and the camera image to predict it.
            '''
            rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
            rospy.Subscriber('/image_color', Image, self.image_cb)

            self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

            self.timestamp_before = None
            self.skipping_duration = None

            rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if not self.waypoints_2d or not self.waypoint_tree:
            return

        # check if it's time to process the next image
        timestamp_now = rospy.get_rostime()
        if (self.timestamp_before == None):
            # it's the very first image
            self.timestamp_before = timestamp_now
            self.skipping_duration = rospy.Duration(SKIPPING_DURATION)
        else:
            time_elapsed = timestamp_now - self.timestamp_before
            if (time_elapsed < self.skipping_duration):
                # do not process this image, wait until enough time has elapsed
                return
            else:
                # set current timestamp and next duration, take jitter into account
                self.timestamp_before = timestamp_now
                self.skipping_duration += rospy.Duration(SKIPPING_DURATION) - time_elapsed

        self.camera_image = msg
        self.has_image = True
        stop_line_wp, state = self.process_traffic_lights()
        # if state == 0:
        # rospy.logwarn("Traffic light: pt=%s, state=%s", stop_line_wp, state)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            stop_line_wp = stop_line_wp if state == TrafficLight.RED else -1
            self.last_wp = stop_line_wp
            self.upcoming_red_light_pub.publish(Int32(stop_line_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # In tests, manually set the lights array and read state from it.
        if self.is_testing:
            return light.state

        if not self.has_image:
            #self.prev_light_loc = None
            rospy.logwarn("Camera not ready yet...")
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")

        # Get classification
        classification = self.light_classifier.get_classification(cv_image)
        self.has_image = False    # Reset; so we only classify new images.
        return classification

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if not self.lights:
            return -1, TrafficLight.UNKNOWN

        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose)

            for i, _ in enumerate(self.lights):
                stop_x, stop_y = self.stop_line_positions[i]
                # Car should stop *before* the stop line.
                stop_line_wp_idx = (self.get_closest_waypoint_xy(stop_x, stop_y) - 1) % len(self.waypoints.waypoints)
                # self.lights is in order. Return as soon as we see the first traffic
                # light (stop line) past the car's position.
                if stop_line_wp_idx > car_wp_idx:
                    if (stop_line_wp_idx - car_wp_idx) < MAX_WP_DISTANCE_TO_CLASSIFY_TL:
                        return stop_line_wp_idx, self.get_light_state(self.lights[i])
                    else:
                        #rospy.logwarn("Request to process traffic light, but it is too far.")
                        return -1, TrafficLight.UNKNOWN

            if len(self.lights):  # waypoint is further than last light - let's loop
                stop_x, stop_y = self.stop_line_positions[0]
                # Car should stop *before* the stop line.
                stop_line_wp_idx = (self.get_closest_waypoint_xy(stop_x, stop_y) - 1) % len(self.waypoints.waypoints)
                if (stop_line_wp_idx - car_wp_idx) % len(self.waypoints.waypoints) < MAX_WP_DISTANCE_TO_CLASSIFY_TL:
                    return stop_line_wp_idx, self.get_light_state(self.lights[0])
                else:
                    #rospy.logwarn("Request to process traffic light, but it is too far.")
                    return -1, TrafficLight.UNKNOWN

        return -1, TrafficLight.UNKNOWN


    # Waypoint updater imported

    def get_closest_waypoint(self, pose):
        x = pose.position.x
        y = pose.position.y
        return self.get_closest_waypoint_xy(x, y)

    def get_closest_waypoint_xy(self, x, y):
        return self.get_closest_array_idx_xy(self.waypoints_2d, self.waypoint_tree, x, y)

    @staticmethod
    def get_closest_array_idx_xy(points_array, points_tree, x, y):
        closest_idx = points_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle/traffic_lights`
        closest_cord = points_array[closest_idx]
        prev_cord = points_array[closest_idx - 1]

        # Hyperplane through closest_cord
        cl_vect = np.array(closest_cord)
        prev_vect = np.array(prev_cord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(points_array)

        return closest_idx

    @staticmethod
    def distance(waypoints, idx1, idx2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(idx1, idx2 + 1):
            dist += dl(waypoints[idx1].pose.pose.position, waypoints[i].pose.pose.position)
            idx1 = i
        return dist

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
