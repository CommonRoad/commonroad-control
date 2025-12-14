from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class OutputInterfaceIndex(ABC):
    """
    Interface for indices of the output variables.
    """

    dim: int


@dataclass
class OutputInterface(ABC):
    """
    Interface for dataclass objects storing outputs of sensor models.
    """

    @property
    def dim(self) -> int:
        """
        :return: output dimension
        """
        return OutputInterfaceIndex.dim

    @abstractmethod
    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,1)
        """
        pass
