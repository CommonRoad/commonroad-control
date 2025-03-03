from abc import ABC, abstractmethod
import enum
import numpy as np
from typing import Tuple, Union
import casadi as cas

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.utils import rk4_integrator


@enum.unique
class ImplementedVehicleModels(enum.Enum):
    KinematicSingleTrack = "KinematicSingleTrack"
    DynamicSingleTrack = "DynamicSingleTrack"


class VehicleModelInterface(ABC):
    def __init__(self, nx: int, nu: int, dt:float):
        """
        Initialize abstract baseclass.
        :param nx: dimension of the state space
        :param nu: dimension of the input space
        """
        self._nx: int = nx
        self._nu: int = nu
        self._dt: float = dt

        # discretize vehicle model
        self._dynamics_dt, self._jac_dynamics_dt_x, self._jac_dynamics_dt_u = self._discretize()

    @abstractmethod
    def simulate_forward(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass

    def _dynamics_ct(self,
                     x: np.array,
                     u: np.array) \
            -> np.array:
        """
        Continuous-time dynamics of the vehicle model.
        :param x: state (array of dimension [self._nx,1])
        :param u: control input (array of dimension [self._nu,1])
        :return: dynamics function evaluated at (x, u)
        """

        x_next = self._dynamics_cas(x, u)
        x_next = np.reshape(x_next, (1, self._nx), order='F').squeeze()

        return x_next

    @abstractmethod
    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.array],
                      u: Union[cas.SX.sym, np.array]) \
            -> cas.SX.sym:
        pass

    @abstractmethod
    def linearize(self, x: StateInterface, u: InputInterface) -> Tuple[StateInterface, np.array, np.array]:
        pass

    def linearize_dt_at(self, x: StateInterface, u: InputInterface) -> Tuple[np.array, np.array, np.array]:

        # convert state and input to arrays
        x_np = x.convert_to_array()
        u_np = u.convert_to_array()

        # evaluate discretized dynamics at (x,u)
        x_next = self._dynamics_ct(x_np, u_np)

        # evaluate linearized dynamics
        jac_x = self._jac_dynamics_dt_x(x_np)
        jac_u = self._jac_dynamics_dt_u(u_np)

        return x_next, jac_x, jac_u

    @abstractmethod
    def position_to_clcs(self, x: StateInterface) -> StateInterface:
        pass

    @abstractmethod
    def position_to_cartesian(self, x: StateInterface) -> StateInterface:
        pass

    def _discretize(self) -> Tuple[cas.Function, cas.Function, cas.Function]:
        """
        Time-discretization of the vehicle model assuming a constant control input throughout the time interval t in
        [0, dt]
        :return: time-discretized dynamical system (CasADi function) and its Jacobian (CasADi function)
        """

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # discretize dynamics
        x_next = cas.Function(
            "dynamics_dt", [xk, uk], [rk4_integrator(xk, uk, self._dynamics_cas, self._dt)]
        )

        # compute Jacobian of discretized dynamics
        jac_x = cas.Function("jac_dynamics_dt", [xk, uk],
                                  [cas.jacobian(x_next(xk, uk), xk)])
        jac_u = cas.Function("jac_dynamics_dt", [xk, uk],
                                  [cas.jacobian(x_next(xk, uk), uk)])
        return x_next, jac_x, jac_u

    @property
    def state_dimension(self):
        return self._nx

    @property
    def input_dimension(self):
        return self._nu


