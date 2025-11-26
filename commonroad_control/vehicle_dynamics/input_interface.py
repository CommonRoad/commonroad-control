from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class InputInterfaceIndex(ABC):
    """
    Indices of the control inputs.
    """

    dim: int


@dataclass
class InputInterface(ABC):
    """
    Control input interface
    """

    @property
    def dim(self):
        return InputInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: (dim, 1) np.ndarray
        """
        pass
