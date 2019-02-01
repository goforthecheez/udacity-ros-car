from yaw_controller import YawController
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

MIN_SPEED = 0.1

class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.2
        kd = 0.
        min_throttle = 0.
        max_throttle = 0.2
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        self.last_sample_time = 0

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        """
        :param linear_velocity:
        :param angular_velocity:
        :param current_velocity:
        :return: throttle, brake, steer
        """

        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        vel_error = linear_velocity - current_velocity
        cur_time = rospy.get_time()
        sample_time = cur_time - self.last_sample_time
        self.last_sample_time = cur_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        return throttle, 0., steer
