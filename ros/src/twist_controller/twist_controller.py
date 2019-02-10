from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.1

class Controller(object):
    def __init__(self, vehicle_mass, decel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)

        kp = 0.3
        ki = 0.2
        kd = 0.0
        min_throttle = 0.0
        max_throttle = 0.2
        self.throttle_controller = PID(kp, ki, kd, min_throttle, max_throttle)

        self.ts = 0.02 # sample time
        self.tau = 0.3 # 1/(2*pi*tau) = cutoff frequency
        self.vel_lpf = LowPassFilter(self.tau / 10.0, self.ts)
        self.brake_lpf = LowPassFilter(self.tau, self.ts)
        self.brake_lpf.filt(0.0)
        self.brake_lpf_reinit = False

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.last_sample_time = rospy.get_time()

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

        current_velocity = self.vel_lpf.filt(current_velocity)
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)

        vel_error = linear_velocity - current_velocity
        cur_time = rospy.get_time()
        sample_time = cur_time - self.last_sample_time
        self.last_sample_time = cur_time

        throttle = self.throttle_controller.step(vel_error, sample_time)

        if ((linear_velocity < 0.02) and (current_velocity < 0.1)):
            throttle = 0.0
            brake = 700.0 # hold Carla in place
            self.brake_lpf_reinit = True
        elif ((throttle < 0.1) and (vel_error < 0.0)):
            if (self.brake_lpf_reinit == True):
                self.brake_lpf = LowPassFilter(self.tau, self.ts)
                self.brake_lpf.filt(0.0)
                self.brake_lpf_reinit = False
            throttle = 0.0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius
            brake = self.brake_lpf.filt(brake/max(9.81 + vel_error/2.0, 1.0))
        else:
            brake = 0.0
            self.brake_lpf_reinit = True

        # rospy.logdebug('vel error: %6.2f / throttle: %6.2f / brake: %6.2f' % (vel_error, throttle, brake))

        return throttle, brake, steer
