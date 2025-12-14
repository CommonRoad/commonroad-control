import unittest
import numpy as np
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.simulation.uncertainty_model.uniform_distribution import UniformDistribution
# example for uncertainty interface
from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import DIDisturbance

logger_global = configure_toolbox_logging(level=logging.DEBUG)


class TestUniformDistribution(unittest.TestCase):
    """
    Unit test for the class modeling a uniformly distributed uncertainty
    """

    def test_initialization_with_arrays(self):
        """
        Initialization from arrays.
        """
        lb = np.array([0.0, 1.0])
        ub = np.array([1.0, 2.0])
        ud = UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub)

        expected_nominal = 0.5 * (lb + ub)
        np.testing.assert_array_equal(ud.nominal_value, expected_nominal)

    def test_initialization_with_uncertainty_interface_bounds(self):
        """
        Initialization from bounds as instances of uncertainty interface dataclass
        """
        lb = DIDisturbance(position_long=0, position_lat=1, velocity_long=0, velocity_lat=1)
        ub = DIDisturbance(position_long=1, position_lat=2, velocity_long=1, velocity_lat=2)
        ud = UniformDistribution(dim=4, lower_bound=lb, upper_bound=ub)
        expected_nominal = 0.5 * (lb.convert_to_array() + ub.convert_to_array())
        np.testing.assert_array_equal(ud.nominal_value, expected_nominal)


    def test_initialization_with_nominal_value(self):
        """
        Initialization including optional input argument for nominal value.
        """

        # initialization from bounds as lists
        lb = [0.0, 1.0]
        ub = [1.0, 2.0]
        nominal = [0.2, 1.5]

        ud = UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub, nominal_value=nominal)
        np.testing.assert_array_equal(ud.nominal_value, nominal)

        # initialization from uncertainty interface dataclass (double integrator disturbance)
        lb = DIDisturbance(position_long=0, position_lat=0, velocity_long=0, velocity_lat=0)
        ub = DIDisturbance(position_long=1, position_lat=1, velocity_long=1, velocity_lat=1)
        nominal = DIDisturbance(position_long=0.2, position_lat=0.5, velocity_long=0.7, velocity_lat=0.9)
        ud = UniformDistribution(dim=4, lower_bound=lb, upper_bound=ub, nominal_value=nominal)
        np.testing.assert_array_equal(ud.nominal_value, nominal.convert_to_array())

    def test_error_input_args(self):
        """
        Handling of erroneous input arguments.
        """

        # dimension mismatch
        lb = [0.0, 1.0]
        ub = [1.0, 2.0, 3.0]
        with self.assertRaises(ValueError):
            UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub)

        # upper bound not greater than lower bound
        lb = [0.0, 2.0]
        ub = [1.0, 1.5]  # ub < lb in second dimension
        with self.assertRaises(ValueError):
            UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub)

        # user-defined nominal value out of bounds
        lb = [0.0, 1.0]
        ub = [1.0, 2.0]
        nominal = [-0.1, 1.5]  # first element out of bounds
        with self.assertRaises(ValueError):
            UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub, nominal_value=nominal)

    def test_sampling_within_bounds(self):
        """
        Test whether randomly sampled values are within the specified bounds
        """

        # initialization from lists
        lb = [0.0, 1.0]
        ub = [1.0, 2.0]
        ud = UniformDistribution(dim=2, lower_bound=lb, upper_bound=ub)
        for _ in range(100):
            sample = ud.sample_uncertainty()
            self.assertTrue(np.all(sample >= lb))
            self.assertTrue(np.all(sample <= ub))

        # initialization from uncertainty interface dataclass (double integrator disturbance)
        lb = DIDisturbance(position_long=0, position_lat=0, velocity_long=0, velocity_lat=0)
        ub = DIDisturbance(position_long=1, position_lat=1, velocity_long=1, velocity_lat=1)
        ud = UniformDistribution(dim=4, lower_bound=lb, upper_bound=ub)
        for _ in range(100):
            sample = ud.sample_uncertainty()
            self.assertTrue(np.all(sample >= lb.convert_to_array()))
            self.assertTrue(np.all(sample <= ub.convert_to_array()))
