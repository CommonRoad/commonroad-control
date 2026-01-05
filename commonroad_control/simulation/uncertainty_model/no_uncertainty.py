from typing import List, Union

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
)
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)


class NoUncertainty(UncertaintyModelInterface):
    """
    Dummy uncertainty model, e.g., if no disturbance or noise models are employed for simulation.
    """

    def __init__(
        self,
        dim: int,
        *args,
        nominal_value: Union[np.ndarray, List[float], UncertaintyInterface, None] = None,
        **kwargs,
    ):
        """
        Initialize no-uncertainty model. If no user-defined nominal value is provided, the nominal value is set to zero.
        :param dim: dimension of the uncertainty - int
        :param nominal_value: if not None: user-defined nominal value of the uncertainty - array/ list of floats/ instance of class UncertaintyInterface
        """

        # if nominal_value is None, set default value
        if nominal_value is None:
            nominal_value = np.zeros(shape=(dim,))

        super().__init__(dim=dim, nominal_value=nominal_value)

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
