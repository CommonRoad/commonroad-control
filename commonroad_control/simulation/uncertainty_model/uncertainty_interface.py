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
    Abstract base class for noise/disturbance interfaces.
    """

    @property
    def dim(self):
        return UncertaintyInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,1)
        """
        pass
