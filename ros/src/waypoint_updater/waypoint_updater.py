#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Bool, Int32
from scipy.spatial import KDTree
import numpy as np
import sys

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number
MAX_DECEL = 10 # m/s^2 TODO replace with ROS parameter

class WaypointUpdater(object):
    def __init__(self, init_for_test=False):
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.obstacle_wp_id = None # we may consider having this as an array for multiple obstacles later on

        if not init_for_test:

            rospy.init_node('waypoint_updater')

            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

            # topic used to test brake planning
            # rostopic pub -1 /obstacle_id std_msgs/Int32 '400'
            rospy.Subscriber('/obstacle_id', Int32, self.obstacle_cb)

            # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

            self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

            self.periodically_publish_waypoints()

    def periodically_publish_waypoints(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree: #tree is constructed later than base_waypoints are set, prevents races
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                rospy.logdebug('Nearest waypoint: %s', closest_waypoint_idx)
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        return self.get_closest_waypoint_idx_xy(x, y)

    def get_closest_waypoint_idx_xy(self, x, y):
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # Check if closest is ahead or behind vehicle/traffic_lights`
        closest_cord = self.waypoints_2d[closest_idx]
        prev_cord = self.waypoints_2d[closest_idx - 1]

        # Hyperplane through closest_cord
        cl_vect = np.array(closest_cord)
        prev_vect = np.array(prev_cord)
        pos_vect = np.array([x, y])
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = self.plan_lane(closest_idx)
        rospy.logdebug('Next waypoint: %s', lane.waypoints[0])
        self.final_waypoints_pub.publish(lane)

    def plan_lane(self, closest_idx):
        lane = Lane()
        lane.header = self.base_waypoints.header

        last_idx = closest_idx + LOOKAHEAD_WPS
        if self.obstacle_wp_id > closest_idx:
            last_idx = min(closest_idx + LOOKAHEAD_WPS, self.obstacle_wp_id) #TODO fix for circle case

        lane.waypoints = self.base_waypoints.waypoints[closest_idx:last_idx]

        if closest_idx <= self.obstacle_wp_id <= last_idx:
            lane.waypoints = self.decelerate_waypoints(lane.waypoints)

        return lane

    def decelerate_waypoints(self, waypoints):
        """
        Gradually reduce speed of waypoints to full stop. Sqrt function is used right now - not very smooth.
        :param waypoints: list of waypoints which velocities will be decelerate to a full stop
        :return: styx_msgs.msg.Lane with coordinates and desired velocities
        """
        lane = []
        dist_total = self.distance(waypoints, 0, len(waypoints) - 1)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, i, len(waypoints) - 1)
            vel = math.sqrt(dist * waypoints[0].twist.twist.linear.x**2 / dist_total) #TODO: account for MAX_DECEL
            if vel < .1:
                vel = 0
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            # sys.stderr.write('Twist: dist={}, vel={}, final={} \n'.format(dist, vel, p.twist.twist.linear.x))
            lane.append(p)

        return lane

    def pose_cb(self, msg):
        self.pose = msg

        if self.waypoint_tree:
            rospy.logdebug("Current wp_idx=%s", self.get_closest_waypoint_idx())

    def waypoints_cb(self, waypoints):
        rospy.logdebug('CB Basewaypoints are being set %d', len(waypoints.waypoints))
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            rospy.logdebug('CB Basewaypoints: tree constructed ok')

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        """
        :param msg: Int32 idx of the waypoint where we want the car to stop
        """
        self.obstacle_wp_id = msg.data
        rospy.logwarn('Obstacle received idx=%s', self.obstacle_wp_id)

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
