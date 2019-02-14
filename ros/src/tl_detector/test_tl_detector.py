#!/usr/bin/env python

# ## in order to run tests first time follow the procedure below (despite how weird it is)

# catkin_make
# source devel/setup.bash
# touch src/tl_detector/CMakeLists.txt
# catkin_make run_tests

# ## touch is used to trigger CMake execution once again after the first build and is required due to some bug in Catkin
# https://github.com/ros/catkin/issues/974
# Every next run of tests can be triggered by
# catkin_make run_tests
# or just simple
# python -m unittest discover -s src/tl_detector

PKG = 'tl_detector'
import copy
import unittest

from tl_detector import TLDetector
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight
from geometry_msgs.msg import Pose, PoseStamped

import numpy as np

N_WPS = 10 # number of waypoints for test


class TestTlDetector(unittest.TestCase):

    @staticmethod
    def _change_light_colors(tl_detector, new_colors):
        for i, color in enumerate(new_colors):
            tl_detector.lights[i].state = color

    def setUp(self):
        self.tld = TLDetector(is_testing=True)

        base_wps = Lane()

        # Waypoints are at (0.2, 0.2) increments.
        for pt in np.arange(0, N_WPS, 0.2):
            wp = Waypoint()
            wp.pose.pose.position.x = pt
            wp.pose.pose.position.y = pt
            base_wps.waypoints.append(wp)

        self.tld.waypoints_cb(base_wps)

        lights = TrafficLightArray()
        l_arr = [[1.0, 1.0, 0], [2.0, 2.2, 2], [3.0, 2.5, 1], [6.0, 6.0, 0] ]
        # Set stop lines (0.5, 0.5) in front of the traffic light.
        # At runtime, stop_line_positions should be set using the ROS parameter server;
        # so for testing, set values by reach into the tld object.
        self.tld.stop_line_positions = []
        for l in l_arr:
            tl = TrafficLight()
            tl.state = l[2]
            tl.pose.pose.position.x = l[0]
            tl.pose.pose.position.y = l[1]
            lights.lights.append(tl)
            self.tld.stop_line_positions.append([l[0] - 0.5, l[1] - 0.5])
        self.tld.traffic_cb(lights)

    def test_wp_cb(self):
        self.assertEqual(len(self.tld.waypoints.waypoints), N_WPS * 5)
        self.assertEqual(len(self.tld.waypoints_2d), N_WPS * 5)
        self.assertTrue(self.tld.waypoint_tree)

    def test_get_closest_wp(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 3.0

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 15)

    def test_get_closest_wp_nearby(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 2.9

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 15)

    def test_get_closest_wp_nearby_above(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 3.1

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 16)

# traffic_cb tests

    def test_get_tl_no_red(self):
        p = PoseStamped()
        p.pose.position.x = 3.0
        p.pose.position.y = 3.0
        self.tld.pose_cb(p)

        # Turn all the lights green.
        TestTlDetector._change_light_colors(self.tld, [2] * 4)

        light_wp, state = self.tld.process_traffic_lights()

        self.assertEqual(light_wp, 27)
        self.assertEqual(state, 2)

    def test_get_tl_wp_next_light_is_red(self):
        p = PoseStamped()
        p.pose.position.x = 3.0
        p.pose.position.y = 3.0
        self.tld.pose_cb(p)

        light_wp, state = self.tld.process_traffic_lights()

        self.assertEqual(light_wp, 27)
        self.assertEqual(state, 0)

    def test_get_tl_wp_next_light_is_red_but_passed_stop_line(self):
        p = PoseStamped()
        p.pose.position.x = 0.6
        p.pose.position.y = 0.6
        self.tld.pose_cb(p)

        light_wp, state = self.tld.process_traffic_lights()

        self.assertEqual(light_wp, 7)
        self.assertEqual(state, 2)

    def test_get_tl_wp_loop(self):
        p = PoseStamped()
        p.pose.position.x = 10.0
        p.pose.position.y = 10.0
        self.tld.pose_cb(p)

        light_wp, state = self.tld.process_traffic_lights()

        self.assertEqual(light_wp, 2)
        self.assertEqual(state, 0)


if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_tl_detector', TestTlDetector)
