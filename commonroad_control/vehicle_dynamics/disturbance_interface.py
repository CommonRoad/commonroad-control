from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class DisturbanceInterfaceIndex(ABC):
    """
    Indices of the disturbances.
    """
    dim: int


@dataclass
class DisturbanceInterface(ABC):
    """
    Disturbance interface
    """

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: (dim, 1) np.ndarray
        """
        pass