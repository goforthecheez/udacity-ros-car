#!/usr/bin/env python
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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):

    def __init__(self, init_without_ros=False):
        """
        :param init_without_ros: in case class has to run in test environment and we don't want ros sub/pub to fail
        """

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.camera_image = None

        self.lights = []
        self.lights_red_2d = [] # [waypoint_idx, ... ] array of waypoints associated with the red light


        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        # self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # for test purposes in order to reduce amount of published data
        self.last_tl_pub_time = 0

        if not init_without_ros:
            rospy.init_node('tl_detector')

            config_string = rospy.get_param("/traffic_light_config")
            self.config = yaml.load(config_string)

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

        if not self.waypoints_2d:
            return

        red_wps_idx = []
        for l in msg.lights:
            if l.state == TrafficLight.RED:
                l_wp_idx = self.get_closest_waypoint_xy(l.pose.pose.position.x, l.pose.pose.position.y)
                red_wps_idx.append(l_wp_idx)
        red_wps_idx.sort()

        self.lights = msg.lights
        self.lights_red_2d = red_wps_idx

        # for testing without proper detector TODO: remove
        # rospy.logwarn("Update TL point 101 wps={}, pose={}".format(self.waypoints is not None, self.pose is not None))
        if self.waypoints and self.pose and self.last_tl_pub_time < rospy.get_rostime().secs:
            self.last_tl_pub_time = rospy.get_rostime().secs
            # rospy.logwarn("Update TL point")
            self.image_cb(None)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        # if state == 0:
        # rospy.logwarn("Traffic light: pt=%s, state=%s", light_wp, state)

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
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
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
        # HACKHACKHACK: Read traffic light color from `/vehicle/traffic_lights` topic.
        # NOTE: These values are only available within the simulator, not in real life.
        return light.state

        #TODO Implement traffic light classifier and uncomment the original code below.
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # stop_line_positions = self.config['stop_line_positions']
        # if(self.pose):
        #     car_position = self.get_closest_waypoint(self.pose.pose)

        # Find the closest visible traffic light (if one exists)
        #TODO maybe only execute this if a light was actually seen in a camera image
        return self.get_closest_light(self.pose.pose)

    def get_closest_light(self, pose):
        """Identifies the closest traffic light to the given position.

        Args:
            post (Pose): position to match a light to

        int: index of the closest light in self.lights.
        """
        if not self.lights :
            return -1, TrafficLight.UNKNOWN

        x = pose.position.x
        y = pose.position.y
        wp_idx = self.get_closest_waypoint_xy(x, y)

        for l_idx in self.lights_red_2d:
            if l_idx > wp_idx:
                return l_idx, TrafficLight.RED

        if len(self.lights_red_2d):  # waypoint is further than last light - let's loop
            return self.lights_red_2d[0], TrafficLight.RED
        else:
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
