from unittest import TestCase

from twist_controller import Controller
import numpy as np

PKG = 'twist_controller'


class TestTwistController(TestCase):

    def setUp(self):
        v_mass = 1736.35
        decel_limit = -5.0
        wheel_radius = 0.2413
        wheel_base = 2.8498
        steer_ratio = 14.8
        max_lat_accel = 3.0
        max_steer_angle = 8.0
        # self.controller = Controller(v_mass, decel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        self.current_velocity = []
        self.linear_velocity = []

    def test_twist(self):
        pass

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_twist_controller', TestTwistController)
