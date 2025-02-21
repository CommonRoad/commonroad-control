from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class StateInterface(ABC):
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

    @abstractmethod
    def set_values_from_np_array(
            self,
            np_array: np.ndarray
    ) -> None:
        """
        Set values of class from np.ndarray
        """
        pass