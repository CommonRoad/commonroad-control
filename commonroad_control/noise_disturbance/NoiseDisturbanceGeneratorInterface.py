from abc import ABC, abstractmethod

import numpy as np


class NoiseDisturbanceGeneratorInterface(ABC):
    """
    Interface for noise and disturbance generators.
    """
    def __init__(
            self,
            dim: int
    ) -> None:
        self._dim: int = dim

    @property
    def dim(self) -> int:
        """
        :return: dimension of noise
        """
        return self._dim

    @abstractmethod
    def apply(
            self,
            state: np.ndarray,
            *args,
            **kwargs
    ) -> np.ndarray:
        """
        Aplly noise or disturbance
        :param args:
        :param kwargs:
        :return: np.ndarray (dim,)
        """
        pass