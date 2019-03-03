from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import rospy
import time

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

        self.kd_mul = -1.0 #for test purposes
        self.kb = vehicle_mass * wheel_radius
        self.brake_controller = PID(-0.3*self.kb, -0.026*self.kb, -0.0125*self.kb, 0, abs(decel_limit) * vehicle_mass * wheel_radius)

        self.vehicle_mass = vehicle_mass
        self.decel_limit = decel_limit
        self.wheel_radius = wheel_radius

        self.last_sample_time = rospy.get_time()

        # self.csv_log = open("/media/sf_Capstone-Team/twist_controller_log-{}.csv".format(time.time()), 'w')
        # self.csv_log.write("Time, v_linear, v_current, v_err, throttle, brake\n")

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

        throttle = self.throttle_controller.step(vel_error, sample_time) if vel_error > 0.0 else 0.0

        if (linear_velocity < 0.02) and (current_velocity < 0.1):
            throttle = 0.0
            brake = 700.0 # hold Carla in place
            self.brake_controller.reset()
        elif vel_error < -0.2:
            throttle = 0.0
            brake = self.brake_controller.step(vel_error, sample_time)
        else:
            brake = 0.0
            self.brake_controller.reset()

        # rospy.logdebug('vel error: %6.2f / throttle: %6.2f / brake: %6.2f' % (vel_error, throttle, brake))
        # self.csv_log.write("{}, {}, {}, {}, {}, {}, {}\n".format(cur_time, linear_velocity, current_velocity, vel_error, throttle, brake, self.kd_mul))

        return throttle, brake, steer

    def debug_inc(self):
        self.kd_mul -= 0.5
        # self.brake_controller.kd = self.kd_mul*self.kb
