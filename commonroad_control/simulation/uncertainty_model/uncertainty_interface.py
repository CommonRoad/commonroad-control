from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class UncertaintyInterfaceIndex(ABC):
    """
    Indices of the uncertainties.
    """

    dim: int


@dataclass
class UncertaintyInterface(ABC):
    """
    Interface for dataclasses storing noise/disturbances.
    """

    @property
    def dim(self):
        """
        :return: uncertainty dimension - int
        """
        return UncertaintyInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,1)
        """
        pass
