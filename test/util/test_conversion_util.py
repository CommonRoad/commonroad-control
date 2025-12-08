import unittest
import logging

import numpy as np

from commonroad_control.util.conversion_util import unwrap_angle
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

logger_global = configure_toolbox_logging(level=logging.DEBUG)

class TestConversionUtil(unittest.TestCase):
    """
    Tests helper functions for state conversion.
    """


    def test_unwrap_angle(self) -> None:
        """
        Unwrapping of angles to avoid discontinuity, when transitioning from pi to -pi in counter-clockwise direction.
        :return:
        """
        # test case 1: no need to wrap
        alpha_prev = 1.0
        alpha_next = 1.1
        out = unwrap_angle(alpha_prev, alpha_next)
        # expected result: out = alpha_next
        assert np.isclose(out, alpha_next, atol=1e-12)

        # test case 2: wrap across positive pi boundary
        alpha_prev = 0.9*np.pi
        alpha_next = -0.9*np.pi
        out = unwrap_angle(alpha_prev, alpha_next)
        # Expected: out = 1.1*pi
        assert np.isclose(out, 1.1*np.pi, atol=1e-12)

        # test case 3: wrap across negative pi boundary
        alpha_prev = -0.9*np.pi
        alpha_next = 0.9*np.pi
        out = unwrap_angle(alpha_prev, alpha_next)
        # Expected: out = -1.1*pi
        assert np.isclose(out, -1.1*np.pi, atol=1e-12)

        # test case 4: large jump
        alpha_prev = 0.0
        alpha_next = 4*np.pi
        out = unwrap_angle(alpha_prev, alpha_next)
        # Expected: out = alpha_prev
        assert np.isclose(out, alpha_prev, atol=1e-12)

        # test case 5: angles differing by pi should map to a difference of +-pi
        alpha_prev = 0.0
        alpha_next = -np.pi
        out = unwrap_angle(alpha_prev, alpha_next)
        # Expected: -pi, exactly (special case)
        assert np.isclose(out, alpha_next, atol=1e-12)

