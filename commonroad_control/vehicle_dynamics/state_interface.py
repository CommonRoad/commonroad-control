from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StateInterfaceIndex(ABC):
    """
    Indices of the states.
    """


@dataclass
class StateInterface(ABC):
    """
    State interface.
    """
    dim: int

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,1)
        """
        pass

    @abstractmethod
    def set_values_from_np_array(
            self,
            x: np.ndarray
    ) -> None:
        """
        Set values of class from a given array.
        :param x: state vector - array of dimension (dim,1)
        """
        pass
