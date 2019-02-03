#!/usr/bin/env python

# ## in order to run tests first time follow the procedure below (despite how weird it is)

# catkin_make
# source devel/setup.bash
# touch src/waypoint_updater/CMakeLists.txt
# catkin_make run_tests

# ## touch is used to trigger CMake execution once again after the first build and is required due to some bug in Catkin
# https://github.com/ros/catkin/issues/974
# Every next run of tests can be triggere by
# catkin_make run_tests
# or just simple
# python -m unittest discover -s src/waypoint_updater

PKG = 'waypoint_updater'
import unittest

from waypoint_updater import WaypointUpdater

from styx_msgs.msg import Lane, Waypoint


class TestDeceleration(unittest.TestCase):

    def setUp(self):
        self.base_wps = Lane()
        pts = [[.0, .0], [1.0, 1.0],[2.0, 2.0],[3.0, 3.0],[4.0, 4.0]]

        self.wpu = WaypointUpdater(init_for_test=True)
        self.wpu.base_waypoints = self.base_wps

        for pt in pts:
            wp = Waypoint()
            wp.pose.pose.position.x = pt[0]
            wp.pose.pose.position.y = pt[1]

            wp.twist.twist.linear.x = 5.3

            self.base_wps.waypoints.append(wp)

    def test_no_decel(self):
        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), 5)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[3].twist.twist.linear.x, lane.waypoints[4].twist.twist.linear.x)

    def test_decel_basic(self):
        self.wpu.obstacle_wp_id = 4

        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), 4)
        self.assertGreater(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertGreater(lane.waypoints[2].twist.twist.linear.x, lane.waypoints[3].twist.twist.linear.x)

    def test_decel_too_far(self):
        self.wpu.obstacle_wp_id = 1000 # needs to be further than Look ahead

        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), 5)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[3].twist.twist.linear.x, lane.waypoints[4].twist.twist.linear.x)


    def test_decel_missed(self):
        self.wpu.obstacle_wp_id = 0

        lane = self.wpu.plan_lane(1)

        self.assertEqual(len(lane.waypoints), 4)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[2].twist.twist.linear.x, lane.waypoints[3].twist.twist.linear.x)


if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_deceleration', TestDeceleration)
