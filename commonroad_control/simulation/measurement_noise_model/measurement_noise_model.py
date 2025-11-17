import numpy as np
from typing import Union, Optional

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface


class MeasurementNoiseModel:
    def __init__(self,
                 uncertainty_model: UncertaintyModelInterface):

        self._uncertainty_model = uncertainty_model

    @property
    def dim(self):
        return self._uncertainty_model.dim

    @property
    def nominal_value(self) -> np.ndarray:
        """
        Nominal value of the underlying uncertainty model.
        :return:
        """
        return self._uncertainty_model.nominal_value

    def apply_noise(self,
                    x_meas: Union[np.array, StateInterface],
                    apply_rand_noise: Optional[bool] = True
                    ) -> np.ndarray:
        """
        Draws a random sample from the underlying uncertainty model and applies it to the measured state.
        :param x_meas: measured state
        :param apply_rand_noise: if true, noise is randomly sampled, otherwise, the nominal value is applied
        :return:
        """
        # convert measured states to numpy array
        if isinstance(x_meas, StateInterface):
            x_meas_np: np.ndarray = x_meas.convert_to_array()
        else:
            x_meas_np= x_meas

        # consistency check
        if self.dim != len(x_meas_np):
            raise ValueError(f"Dimension mismatch: "
                             f"dim. of noise:{self.dim}, dim. of measured state: {len(x_meas_np)}")

        # sample and apply noise
        if apply_rand_noise:
            noise_np = self._uncertainty_model.sample_uncertainty()
        else:
            noise_np = self._uncertainty_model.nominal_value
        x_noisy = x_meas_np + noise_np

        return x_noisy
