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
from std_msgs.msg import Int32

N_WPS = 10


class TestDeceleration(unittest.TestCase):

    def setUp(self):
        self.base_wps = Lane()

        self.wpu = WaypointUpdater(init_for_test=True)
        self.wpu.base_waypoints = self.base_wps

        for pt in range(N_WPS):
            wp = Waypoint()
            wp.pose.pose.position.x = pt
            wp.pose.pose.position.y = pt

            wp.twist.twist.linear.x = 5.3

            self.base_wps.waypoints.append(wp)

    def test_no_decel(self):
        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), N_WPS)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[3].twist.twist.linear.x, lane.waypoints[4].twist.twist.linear.x)

    def test_decel_basic(self):
        self.wpu.obstacle_wp_id = 7

        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), 7)
        self.assertGreater(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertGreater(lane.waypoints[1].twist.twist.linear.x, lane.waypoints[2].twist.twist.linear.x)

    # obstale waypoint is far away
    def test_decel_too_far(self):
        self.wpu.obstacle_wp_id = 1000 # needs to be further than Look ahead

        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), N_WPS)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[3].twist.twist.linear.x, lane.waypoints[4].twist.twist.linear.x)

    # obstacle point is before closest waypoint (wp)
    def test_decel_missed(self):
        self.wpu.obstacle_wp_id = 0

        lane = self.wpu.plan_lane(1)

        self.assertEqual(len(lane.waypoints), N_WPS - 1)
        self.assertEqual(lane.waypoints[0].twist.twist.linear.x, lane.waypoints[1].twist.twist.linear.x)
        self.assertEqual(lane.waypoints[2].twist.twist.linear.x, lane.waypoints[3].twist.twist.linear.x)

    # 2 last waypoints should have speed equal to zero
    def test_stop_2_till_end(self):
        obtacle_idx = 10
        self.wpu.obstacle_wp_id = obtacle_idx

        lane = self.wpu.plan_lane(0)

        self.assertEqual(len(lane.waypoints), obtacle_idx)
        self.assertEqual(lane.waypoints[obtacle_idx - 1].twist.twist.linear.x, 0.0)
        self.assertEqual(lane.waypoints[obtacle_idx - 1].twist.twist.linear.x, lane.waypoints[obtacle_idx - 2].twist.twist.linear.x)

        # self.assertGreater(lane.waypoints[2].twist.twist.linear.x, lane.waypoints[3].twist.twist.linear.x)

    def test_obstacle_reset(self):
        self.wpu.obstacle_wp_id = 3

        msg = Int32()
        msg.data = None
        self.wpu.traffic_cb(msg)

        self.assertFalse(self.wpu.obstacle_wp_id)


    def test_obstacle_reset_1(self):
        self.wpu.obstacle_wp_id = 3

        msg = Int32()
        msg.data = -1
        self.wpu.traffic_cb(msg)

        self.assertFalse(self.wpu.obstacle_wp_id)

    def test_obstacle_set(self):
        self.wpu.obstacle_wp_id = None

        msg = Int32()
        msg.data = 3
        self.wpu.traffic_cb(msg)

        self.assertEqual(self.wpu.obstacle_wp_id, 3)

if __name__ == '__main__':
   import rosunit
   rosunit.unitrun(PKG, 'test_deceleration', TestDeceleration)
