from abc import ABC, abstractmethod, abstractproperty
import numpy as np


class UncertaintyModelInterface(ABC):
    def __init__(self,
                 dim: int):
        self._dim: int = dim

    @property
    def dim(self) -> int:
        return self._dim

    @property
    @abstractmethod
    def nominal_value(self) -> np.ndarray:
        pass

    @abstractmethod
    def sample_uncertainty(self):
        pass
