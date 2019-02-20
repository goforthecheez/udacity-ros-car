#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PointStamped
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

LOOKAHEAD_WPS = 70  # Number of waypoints we will publish. You can change this number
DECEL_AT_STOP = -0.3 # how fast shall we decelerate at the last point, m/s
DECEL_AT_START = -0.05
WPS_OFFSET_INFRONT_CAR = 4

WPU_UPDATE_FREQUENCY = 20

class WaypointUpdater(object):
    def __init__(self, init_for_test=False):
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.obstacle_wp_id = None # we may consider having this as an array for multiple obstacles later on

        self.plot_time = 0 # for debug purposes

        if not init_for_test:
            rospy.init_node('waypoint_updater')

            rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
            rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

            # to manualy publish: rostopic pub -1 /traffic_waypoint std_msgs/Int32 '400'
            rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

            self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

            self.periodically_publish_waypoints()

    def periodically_publish_waypoints(self):
        rate = rospy.Rate(WPU_UPDATE_FREQUENCY)
        while not rospy.is_shutdown():
            if self.pose and self.waypoint_tree: #tree is constructed later than base_waypoints are set, prevents races
                t1 = rospy.get_rostime()
                # Get closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                # rospy.logdebug('Nearest waypoint: %s', closest_waypoint_idx)
                self.publish_waypoints(closest_waypoint_idx)

                t2 = rospy.get_rostime()
                # rospy.logwarn('WPU runtime = %f', (t2-t1).to_sec())
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
        lane = self.plan_lane(closest_idx, rospy.get_rostime())
        self.final_waypoints_pub.publish(lane)

    def plan_lane(self, closest_idx, time=0):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.header.stamp = time
        lane.header.seq = closest_idx

        last_idx = closest_idx + LOOKAHEAD_WPS
        if self.obstacle_wp_id > closest_idx:
            last_idx = min(last_idx, self.obstacle_wp_id)

        lane.waypoints = self.base_waypoints.waypoints[closest_idx + WPS_OFFSET_INFRONT_CAR:last_idx]
        # rospy.logwarn("Last wps: %s", last_idx)

        if closest_idx <= self.obstacle_wp_id <= last_idx:
            lane.waypoints = self.decelerate_waypoints(lane.waypoints)
            # if self.plot_time < rospy.get_rostime().secs:
            #     self.plot_velocity(lane, rospy.get_rostime())
            #     self.plot_time = rospy.get_rostime().secs


        return lane

    def decelerate_waypoints(self, waypoints):
        """
        Gradually reduce speed of waypoints to full stop. Sqrt function is used right now - not very smooth.
        :param waypoints: list of waypoints which velocities will be decelerate to a full stop
        :return: styx_msgs.msg.Lane with coordinates and desired velocities
        """
        lane = []
        if not waypoints or len(waypoints) == 0:
            return lane

        pts_before_end = 2 # leave several points ahead as we are counting from the middle of the car
        total_dist = self.distance(waypoints, 0, len(waypoints) - pts_before_end - 1) # total distance to obstacle
        start_vel = waypoints[0].twist.twist.linear.x #velocity of the car at the start of trajectory
        start_vel_kmh = start_vel * 3.6
        stop_d = start_vel_kmh * 2.0 # just an assumption that safe stop distance is equal to your speed value (but in meters)

        start_params = [start_vel, DECEL_AT_START, 0] # velocity, acceleration, jerk - derivatives of velocity
        coefs = self.gen_jerk_safe_poly(stop_d, start_params, [0, DECEL_AT_STOP, 0])

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            dist = self.distance(waypoints, 0, i)

            vel = self.jerk_safe_polynom_value(dist - (total_dist - stop_d), coefs)

            if vel < 0.05:
                vel = 0
            vel = min(vel, wp.twist.twist.linear.x)
            p.twist.twist.linear.x = vel
            # sys.stderr.write('Twist: dist={}, vel={}, final={} \n'.format(dist, vel, p.twist.twist.linear.x))
            lane.append(p)

        return lane

    def jerk_safe_polynom_value(self, d, coefs):
        return coefs[0] + coefs[1]*d + coefs[2]*d**2 + coefs[3]*d**3 + coefs[4]*d**4 + coefs[5]*d**5

    def gen_jerk_safe_poly(self, T, start, end):
        """
        Calculate cooficients of the 5th degree polinom that satisfies conditions at the start of trajectory
        and at the end.
        a0 + a1*d + a2*d^2 + a3*d^3 + a4*d^4 + a5*d^5

        :param T: finish point for polinom. Start is at zero
        :param start: parameters at start [velocity, acceleration, jerk]
        :param end: parameters at end point
        :return: 6 coefficients for the polinom
        """
        T3 = T*T*T
        T4 = T*T*T*T
        T5 = T*T*T*T*T

        a = np.array([[T3, T4, T5],[3*T*T, 4*T3, 5*T4],[6*T, 12*T*T, 20*T3]])

        b = np.array([end[0] - (start[0] + start[1]*T + 0.5*start[2]*T*T),
                         end[1] - (start[1] + start[2]*T),
                         end[2] - start[2]])


        x = np.linalg.solve(a, b)
        # for xi in x:
        #     print("{:.6f}".format(xi.item()))
        return [start[0], start[1], start[2], x[0], x[1], x[2]]

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
        """
        :param msg: Int32 idx of the waypoint where we want the car to stop
        """
        if msg.data and msg.data >= 0:
            self.obstacle_wp_id = msg.data
            rospy.logwarn('Obstacle received idx=%s', self.obstacle_wp_id)
        else:
            self.obstacle_wp_id = None

        ## debug info
        # t = rospy.get_rostime()
        # ln = self.plan_lane(self.get_closest_waypoint_idx(), t)

        # self.plot_velocity(ln, t)

    def plot_velocity(self, ln, t):
        """
        Plot velocities of a waypoint lane (twist.twist.linear.x).
        Plot will be save to ROS log folder.
        :param ln: lane object with waypoints
        :param t: ros time
        """
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots( nrows=1, ncols=1 )  # create figure & 1 axis

        vel = []
        dist = []
        n = len(ln.waypoints)
        for i, wp in enumerate(ln.waypoints):
            vel.append(wp.twist.twist.linear.x)
            dist.append(self.distance(ln.waypoints, 0, i))
        ax.plot(dist, vel)
        fig.savefig('/home/student/.ros/log/vel_dist_{}.png'.format(t.secs))  # save the figure to file
        plt.close(fig)

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
