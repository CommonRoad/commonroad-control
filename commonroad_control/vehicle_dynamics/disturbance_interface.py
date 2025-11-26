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
    Indices of the disturbances.
    """

    dim: int


@dataclass
class DisturbanceInterface(UncertaintyInterface):
    """
    Baseclass for disturbance interfaces.
    """

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: (dim, 1) np.ndarray
        """
        pass
