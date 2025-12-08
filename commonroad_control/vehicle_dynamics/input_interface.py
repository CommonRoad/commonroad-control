from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class InputInterfaceIndex(ABC):
    """
    Interface for the indices of the control inputs.
    """

    dim: int


@dataclass
class InputInterface(ABC):
    """
    Interface for dataclass objects storing control inputs of the vehicle models.
    """

    @property
    def dim(self) -> int:
        """
        :return: control input dimension
        """
        return InputInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: (dim, 1) np.ndarray
        """
        pass
