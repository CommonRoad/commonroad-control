from abc import abstractmethod
from dataclasses import dataclass

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
    UncertaintyInterfaceIndex,
)


@dataclass(frozen=True)
class FullStateNoiseInterfaceIndex(UncertaintyInterfaceIndex):
    """
    Indices of the noise variables.
    """

    dim: int


@dataclass
class FullStateNoiseInterface(UncertaintyInterface):
    """
    Abstract base class for full state noise - required for the full state feedback sensor model.
    """

    @property
    def dim(self):
        return FullStateNoiseInterface.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,1)
        """
        pass
