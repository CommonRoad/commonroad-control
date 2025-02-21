from abc import ABC, abstractmethod
import enum
import numpy as np
from typing import Tuple, Callable
import casadi as cas

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.utils import rk4_integrator


@enum.unique
class ImplementedModels(enum.Enum):
    KinematicSingleTrack = "KinematicSingleTrack"
    DynamicSingleTrack = "DynamicSingleTrack"


class VehicleModelInterface(ABC):
    def __init__(self, nx: int, nu: int, dt=float):
        """
        Initialize abstract baseclass.
        :param nx: dimension of the state space
        :param nu: dimension of the input space
        """
        self._nx = nx
        self._nu = nu

        # discretize vehicle model
        self._dynamics_dt = self._discretize(dt)

    @abstractmethod
    def simulate_forward(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass

    @abstractmethod
    def _dynamics(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass

    def _discretize(self, dt: float) -> cas.Function:
        """
        Time-discretization of the vehicle model assuming a constant control input throughout the time interval t in
        [0, dt]
        :param dt: sampling time
        :return: time-discretized dynamical system
        """

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        x_next = cas.Function(
            "dynamics_dt", [xk, uk], [rk4_integrator(xk, uk, self._dynamics, dt)]
        )

        return x_next

    @abstractmethod
    def linearize(self, x: StateInterface, u: InputInterface) -> Tuple[StateInterface, np.array, np.array]:
        pass

    @abstractmethod
    def position_to_clcs(self, x: StateInterface) -> StateInterface:
        pass

    @abstractmethod
    def position_to_cartesian(self, x: StateInterface) -> StateInterface:
        pass
