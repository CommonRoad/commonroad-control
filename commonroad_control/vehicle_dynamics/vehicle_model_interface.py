from abc import ABC, abstractmethod
import enum
import numpy as np
from typing import Tuple

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface


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
    def simulate_forward(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass

    @abstractmethod
    def discretize(self, x: StateInterface, u: InputInterface) -> InputInterface:
        pass

    @abstractmethod
    def linearize(self, x: StateInterface, u: InputInterface) -> Tuple[StateInterface, np.array, np.array]:
        pass

    @abstractmethod
    def position_to_clcs(self, x: StateInterface) -> StateInterface:
        pass

    @abstractmethod
    def position_to_cartesian(self, x: StateInterface) -> StateInterface:
        pass
