#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd', SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd', ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd', BrakeCmd, queue_size=1)

        self.dbw_enabled = False
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.current_velocity = 0 # 0 might not be the best in case if we enable dbw on the go - for the future

        # TODO: Create `Controller` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                                     wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        if not self.dbw_enabled:
            rospy.logwarn('DBW is OFF. If you are running on simulator - just turn the Manual mode OFF to enable DBW')
        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            throttle, brake, steer = self.controller.control(self.linear_velocity, self.angular_velocity,
                                                             self.current_velocity, self.dbw_enabled)
            # <proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            if self.dbw_enabled:
                rospy.logdebug('Steer_cmd: steer=%s, lin_vel=%s, ang_vel=%s, cur_vel=%s', steer, self.linear_velocity, self.angular_velocity, self.current_velocity)
                self.publish_controls(throttle, brake, steer)
            rate.sleep()

    def publish_controls(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    # ######### Callback functions ############

    def twist_cb(self, msg):
        """

        :param msg: geometry_msgs/TwistStamped
            Vector3 linear
                float64 x
                float64 y
                float64 z
            Vector3 angular
                float64 x
                float64 y
                float64 z
        """
        self.linear_velocity = msg.twist.linear.x
        self.angular_velocity = msg.twist.angular.z
        # rospy.logdebug('CB twist linear=%s, angular=%s', self.linear_velocity, self.angular_velocity)

    def dbw_enabled_cb(self, is_enabled):
        rospy.logwarn('CB dbw_enabled=%s', is_enabled.data)
        self.dbw_enabled = is_enabled.data

    def current_velocity_cb(self, msg):
        """
        :param msg:  geometry_msgs/TwistStamped
        """
        self.current_velocity = msg.twist.linear.x
        # rospy.logdebug('CB velocity=%s', self.current_velocity)

if __name__ == '__main__':
    DBWNode()
