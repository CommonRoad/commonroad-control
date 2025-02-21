from abc import ABC, abstractmethod
import enum
import numpy as np


@enum.unique
class ImplementedModels(enum.Enum):
    KinematicSingleTrack = "KinematicSingleTrack"
    DynamicSingleTrack = "DynamicSingleTrack"


class VehicleModelInterface(ABC):
    def __init__(self, nx: int, nu: int):
        """
        Initialize abstract baseclass.
        :param nx: dimension of the state space
        :param nu: dimension of the input space
        """
        self._nx = nx
        self._nu = nu

    @abstractmethod
    def simulate_forward(self, x, u):
        pass

    @abstractmethod
    def discretize(self,x,u):
        pass

    @abstractmethod
    def linearize(self, x, u) -> (np.array, np.array):
        pass

    @abstractmethod
    def position_to_clcs(self, x):
        pass

    @abstractmethod
    def position_to_cartesian(self, x):
        pass
