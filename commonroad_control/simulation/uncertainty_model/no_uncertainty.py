import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)


class NoUncertainty(UncertaintyModelInterface):
    """
    Dummy uncertainty model, e.g., if no disturbance or noise models are employed for simulation.
    """

    def __init__(self, dim: int, *args, **kwargs):

        super().__init__(dim=dim)
        self._nominal_value = np.zeros(shape=(self._dim,))

    @property
    def nominal_value(self) -> np.ndarray:
        """
        Returns the nominal value of uncertainty model.
        :return: np.ndarray of dimension (self.dim,)
        """
        return self._nominal_value

    def sample_uncertainty(self) -> np.ndarray:
        """
        Since this model features no uncertainty, the nominal value is returned.
        :return:  np.ndarray of dimension (self.dim,)
        """
        return self.nominal_value
