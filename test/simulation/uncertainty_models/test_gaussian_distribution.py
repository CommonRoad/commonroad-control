import unittest
import numpy as np
from numpy.ma.testutils import assert_equal
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.simulation.uncertainty_model.gaussian_distribution import GaussianDistribution
# example for uncertainty interface
from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import DIDisturbance

logger_global = configure_toolbox_logging(level=logging.INFO)


class TestGaussianDistribution(unittest.TestCase):
    """
    Unit test for the class modeling a normally distributed uncertainty
    """

    def test_initialization_with_arrays(self):
        """
        Initialization from arrays.
        """

        mean = np.array([0.0, 1.0])
        std = np.array([0.1, 0.2])
        gd = GaussianDistribution(dim=2, mean=mean, std_deviation=std)

        np.testing.assert_array_equal(gd.mean, mean)
        np.testing.assert_array_equal(gd.std_deviations, std)
        np.testing.assert_array_equal(gd.nominal_value, mean)

    def test_initialization_with_uncertainty_interface_bounds(self):
        """
        Initialization from bounds as instances of uncertainty interface dataclass
        """

        mean = DIDisturbance(position_long=0, position_lat=1, velocity_long=0, velocity_lat=1)
        std = DIDisturbance(position_long=0.1, position_lat=0.2, velocity_long=0.3, velocity_lat=0.4)
        gd = GaussianDistribution(dim=4, mean=mean, std_deviation=std)

        np.testing.assert_array_equal(gd.mean, mean.convert_to_array())
        np.testing.assert_array_equal(gd.std_deviations, std.convert_to_array())
        np.testing.assert_array_equal(gd.nominal_value, mean.convert_to_array())

    def test_initialization_with_nominal_value(self):
        """
        Initialization including optional input argument for nominal value.
        """

        # initialization from parameters as lists
        mean = [0.0, 1.0]
        std = [0.1, 0.2]
        nominal = [0.2, 1.5]
        gd = GaussianDistribution(dim=2, mean=mean, std_deviation=std, nominal_value=nominal)

        np.testing.assert_array_equal(gd.nominal_value, nominal)

        # initialization from uncertainty interface dataclass (double integrator disturbance)
        mean = DIDisturbance(position_long=0, position_lat=0, velocity_long=0, velocity_lat=0)
        std = DIDisturbance(position_long=0.1, position_lat=0.2, velocity_long=0.3, velocity_lat=0.4)
        nominal = DIDisturbance(position_long=0.05, position_lat=0.1, velocity_long=0.2, velocity_lat=0.3)
        gd = GaussianDistribution(dim=4, mean=mean, std_deviation=std, nominal_value=nominal)

        np.testing.assert_array_equal(gd.nominal_value, nominal.convert_to_array())

    def test_error_input_args(self):
        """
        Handling of erroneous input arguments.
        """

        # dimension mismatch
        mean = [0.0, 1.0]
        std = [0.1, 0.2, 0.3]
        with self.assertRaises(ValueError):
            GaussianDistribution(dim=2, mean=mean, std_deviation=std)

        # negative values of standard deviation
        mean = [0.0, 1.0]
        std = [0.1, -0.2]  # negative std
        with self.assertRaises(ValueError):
            GaussianDistribution(dim=2, mean=mean, std_deviation=std)

    def test_sample_uncertainty_within_expected_range(self):
        """
        Test random sampling.
        """

        # initialization from lists
        mean = [0.0, 0.0]
        std = [0.1, 0.2]
        gd = GaussianDistribution(dim=2, mean=mean, std_deviation=std)

        sample = gd.sample_uncertainty()
        assert isinstance(sample, np.ndarray)
        assert_equal(sample.shape, (gd.dim,))

        # initialization from uncertainty interfaces
        mean = DIDisturbance(position_long=0, position_lat=0, velocity_long=0, velocity_lat=0)
        std = DIDisturbance(position_long=0.1, position_lat=0.2, velocity_long=0.3, velocity_lat=0.4)
        gd = GaussianDistribution(dim=4, mean=mean, std_deviation=std)

        sample = gd.sample_uncertainty()
        assert isinstance(sample, np.ndarray)
        assert_equal(sample.shape, (gd.dim,))
