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
import unittest

from tl_detector import TLDetector
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight
from geometry_msgs.msg import PoseStamped, Pose

N_WPS = 10 # number of waypoints for test


class TestTlDetector(unittest.TestCase):

    def setUp(self):
        self.tld = TLDetector(is_testing=True)

        base_wps = Lane()

        for pt in range(N_WPS):
            wp = Waypoint()
            wp.pose.pose.position.x = pt
            wp.pose.pose.position.y = pt
            base_wps.waypoints.append(wp)

        self.tld.waypoints_cb(base_wps)

        lights = TrafficLightArray()
        l_arr = [[1.0, 1.0, 0], [2.0, 2.2, 4], [3.0, 2.5, 1], [6.0, 6.0, 0] ]
        for l in l_arr:
            tl = TrafficLight()
            tl.state = l[2]
            tl.pose.pose.position.x = l[0]
            tl.pose.pose.position.y = l[1]
            lights.lights.append(tl)
        self.tld.traffic_cb(lights)
        self.tld.stop_line_positions = [[p[0], p[1]] for p in l_arr] # just put the stop line at the traffic light

    def test_wp_cb(self):
        self.assertEqual(len(self.tld.waypoints.waypoints), N_WPS)
        self.assertEqual(len(self.tld.waypoints_2d), N_WPS)
        self.assertTrue(self.tld.waypoint_tree)

    def test_get_closest_wp(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 3.0

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 3)

    def test_get_closest_wp_nearby(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 2.5

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 3)

    def test_get_closest_wp_nearby_above(self):
        p = Pose()
        p.position.x = 3.0
        p.position.y = 3.5

        idx = self.tld.get_closest_waypoint(p)

        self.assertEqual(idx, 4)

# traffic_cb tests
    def test_get_tl(self):
        self.assertEqual(self.tld.lights_red_idxs, [0, 3]) #corresponding waypoint ids for each red traffic light

    def test_get_tl_no_red(self):
        p = PoseStamped()
        p.pose.position.x = 3.0
        p.pose.position.y = 3.0

        self.tld.lights_red_idxs = []
        # self.tld.lights = []

        idx = self.tld.process_traffic_lights()

        self.assertEqual(idx, (-1, 4))

    def test_get_tl_wp_1ahead(self):
        p = PoseStamped()
        p.pose.position.x = 3.0
        p.pose.position.y = 3.0
        self.tld.pose = p

        idx = self.tld.process_traffic_lights()

        self.assertEqual(idx, (6, 0))

    def test_get_tl_wp_2ahead(self):
        p = PoseStamped()
        p.pose.position.x = 0.0
        p.pose.position.y = 0.1
        self.tld.pose = p

        idx = self.tld.process_traffic_lights()

        self.assertEqual(idx, (1, 0))

    def test_get_tl_wp_loop(self):
        p = PoseStamped()
        p.pose.position.x = 10.0
        p.pose.position.y = 10.0
        self.tld.pose = p

        idx = self.tld.process_traffic_lights()

        self.assertEqual(idx, (1, 0))



if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_tl_detector', TestTlDetector)
