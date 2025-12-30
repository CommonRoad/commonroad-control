from abc import ABC, abstractmethod
from typing import List, Union

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
)


class UncertaintyModelInterface(ABC):
    """
    Interface for uncertainty models for modeling disturbances or sensor noise. Examples include the gaussian or uniform distribution.
    """

    def __init__(
        self,
        dim: int,
        nominal_value: Union[np.ndarray, List[float], UncertaintyInterface],
    ) -> None:
        """
        Initialize uncertainty model.
        :param dim: dimension of the uncertainty - int
        :param nominal_value: (optinally user-defined) nominal value of the uncertainty - array/ list of floats/ instance of class UncertaintyInterface
        """

        self._dim: int = dim

        # set nominal value
        if isinstance(nominal_value, UncertaintyInterface):
            nominal_value_np = nominal_value.convert_to_array()
        else:
            nominal_value_np: np.ndarray = np.array(nominal_value)
        self._nominal_value = nominal_value_np

    @property
    def dim(self) -> int:
        """
        :return: dimension of the uncertainty
        """
        return self._dim

    @property
    @abstractmethod
    def nominal_value(self) -> np.ndarray:
        """
        Returns the nominal value of uncertainty model (the expected value of the underlying distribution or a user-defined nominal value).
        :return: np.ndarray of dimension (self.dim,)
        """
        pass

    @abstractmethod
    def sample_uncertainty(self):
        pass
