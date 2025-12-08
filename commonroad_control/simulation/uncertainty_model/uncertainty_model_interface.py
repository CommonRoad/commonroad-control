from abc import ABC, abstractmethod

import numpy as np


class UncertaintyModelInterface(ABC):
    """
    Interface for uncertainty models for modelling disturbances or sensor noise. Examples include the gaussian or uniform distribution.
    """

    def __init__(self, dim: int) -> None:
        self._dim: int = dim

    @property
    def dim(self) -> int:
        """
        :return: dimension of the uncertainty
        """
        return self._dim

    @property
    @abstractmethod
    def nominal_value(self) -> np.ndarray:
        """
        Returns the nominal value of uncertainty model (e.g. the mean for the Gaussian distribution or a user-defined nominal value).
        :return: np.ndarray of dimension (self.dim,)
        """
        pass

    @abstractmethod
    def sample_uncertainty(self):
        pass
