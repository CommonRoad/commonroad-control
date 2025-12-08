from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StateInterfaceIndex(ABC):
    """
    Interface for indices of the state variables.
    """

    dim: int


@dataclass
class StateInterface(ABC):
    """
    Interface for dataclass objects storing states of the vehicle models.
    """

    @property
    def dim(self) -> int:
        """
        :return: state dimension
        """
        return StateInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,1)
        """
        pass
