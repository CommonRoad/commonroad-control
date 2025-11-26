from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StateInterfaceIndex(ABC):
    """
    Indices of the states.
    """

    dim: int


@dataclass
class StateInterface(ABC):
    """
    State interface.
    """

    @property
    def dim(self):
        return StateInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,1)
        """
        pass
