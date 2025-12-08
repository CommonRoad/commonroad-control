from abc import abstractmethod
from dataclasses import dataclass

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
    UncertaintyInterfaceIndex,
)


@dataclass(frozen=True)
class DisturbanceInterfaceIndex(UncertaintyInterfaceIndex):
    """
    Indices of the disturbance variables.
    """

    dim: int


@dataclass
class DisturbanceInterface(UncertaintyInterface):
    """
    Interface for dataclass objects storing disturbances of the vehicle models.
    """

    @property
    def dim(self) -> int:
        """
        :return: disturbance dimension
        """
        return DisturbanceInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        pass
