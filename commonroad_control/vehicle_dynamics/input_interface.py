from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class InputInterfaceIndex(ABC):
    """
    Indices of the control inputs.
    """


@dataclass
class InputInterface(ABC):
    """
    State interface
    """
    dim: int

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts state to numpy array
        :return: (dim, 1) np.ndarray
        """
        pass
