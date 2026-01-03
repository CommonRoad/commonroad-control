import unittest
import numpy as np
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.simulation.uncertainty_model.no_uncertainty import NoUncertainty

logger_global = configure_toolbox_logging(level=logging.INFO)


class TestNoUncertainty(unittest.TestCase):
    """
    Test for dummy uncertainty model (employed for nominal simulation.
    """

    def test_nominal_value_and_sample(self):
        """
        Test instantiation and sampling
        :return:
        """
        dim = 5

        nu = NoUncertainty(dim=dim)

        # Check nominal value (must be zero)
        nominal = nu.nominal_value
        self.assertIsInstance(nominal, np.ndarray)
        self.assertEqual(nominal.shape, (dim,))
        self.assertTrue(np.all(nominal == 0.0))

        # Check sampling -> should return the nominal value
        sample = nu.sample_uncertainty()
        self.assertIsInstance(sample, np.ndarray)
        self.assertEqual(sample.shape, (dim,))
        self.assertTrue(np.all(sample == 0.0))
        np.testing.assert_array_equal(sample, nominal)