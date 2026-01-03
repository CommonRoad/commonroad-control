import unittest
import numpy as np
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import FullStateFeedback
from commonroad_control.simulation.uncertainty_model.uniform_distribution import UniformDistribution

# double integrator state and input for testing
from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import  DISIDTFactory

logger_global = configure_toolbox_logging(level=logging.INFO)


class TestFullStateFeedback(unittest.TestCase):
    """
    Unit test for the full state feedback sensor model.
    """

    def test_measure_with_noise_not_nominal(self):

        dim = DISIDTFactory.state_dimension


        # Create a UniformDistribution as noise model that excludes zero
        lb = 0.1
        lower_bound = np.ones(dim) * 2*lb
        upper_bound = np.ones(dim) * 1.0
        noise_model = UniformDistribution(
            dim=dim,
            lower_bound=lower_bound,
            upper_bound=upper_bound
        )


        # Create the sensor model
        sensor = FullStateFeedback(
            noise_model=noise_model,
            state_output_factory=DISIDTFactory(),
            state_dimension=DISIDTFactory.state_dimension,
            input_dimension=DISIDTFactory.input_dimension
        )

        # Create mock state and input
        x = DISIDTFactory.state_from_numpy_array(np.zeros(DISIDTFactory.state_dimension))
        u = DISIDTFactory.input_from_numpy_array(np.zeros(DISIDTFactory.input_dimension))

        # Measure with random noise - measurement must not coincide with x
        noisy_meas_np = sensor.measure(x, u).convert_to_array()
        self.assertIsInstance(noisy_meas_np, np.ndarray)
        self.assertEqual(noisy_meas_np.shape, (DISIDTFactory.state_dimension,))
        self.assertGreater(np.linalg.norm(
            noisy_meas_np - x.convert_to_array()), lb)

        # measurement with no random noise -> measurement must be the nominal value of the noise model
        nominal_meas_np = sensor.measure(x, u, rand_noise=False).convert_to_array()
        self.assertIsInstance(nominal_meas_np, np.ndarray)
        self.assertEqual(nominal_meas_np.shape, (DISIDTFactory.state_dimension,))
        assert np.isclose(nominal_meas_np, noise_model.nominal_value).all()
