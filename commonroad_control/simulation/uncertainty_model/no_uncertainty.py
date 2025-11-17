import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface


class NoUncertainty(UncertaintyModelInterface):
    def __init__(self,
                 dim: int):

        super().__init__(dim=dim)
        self._nominal_value = np.zeros(shape=(self._dim,))

    @property
    def nominal_value(self):
        return self._nominal_value

    def sample_uncertainty(self):
        return self.nominal_value

