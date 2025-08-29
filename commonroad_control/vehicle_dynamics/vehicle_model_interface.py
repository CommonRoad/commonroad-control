from abc import ABC, abstractmethod
import enum
import inspect
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
    def __init__(self, nx: int, nu: int, delta_t:float):
        """
        Initialize abstract baseclass.
        :param nx: dimension of the state space
        :param nu: dimension of the input space
        :param delta_t: sampling time
        """
        self._nx: int = nx
        self._nu: int = nu
        self._delta_t: float = delta_t

        # discretize vehicle model
        self._dynamics_dt, self._jac_dynamics_dt_x, self._jac_dynamics_dt_u = self._discretize()

        # differentiate acceleration constraint functions
        self._a_norm, self._jac_a_norm_long_x, self._jac_a_norm_long_u, self._jac_a_norm_lat_x, self._jac_a_norm_lat_u, \
            = self._differentiate_acceleration_constraints()

    @abstractmethod
    def simulate_forward(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass

    def simulate_forward_dt(self,
                            x: Union[StateInterface, np.array],
                            u: Union[InputInterface, np.array]) \
            -> np.array:

        # convert state and input to arrays
        if isinstance(x, StateInterface):
            x_np = x.convert_to_array()
        else:
            x_np = x

        if isinstance(u, InputInterface):
            u_np = u.convert_to_array()
        else:
            u_np = u

        # evaluate discretized dynamics at (x,u)
        x_next = self._dynamics_dt(x_np, u_np).full()

        return x_next.squeeze()

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

    def linearize_dt_at(self,
                        x: Union[StateInterface, np.array],
                        u: Union[InputInterface, np.array]) \
            -> Tuple[np.array, np.array, np.array]:
        """
        Linearization of the time-discretized vehicle dynamics at a given state-input-pair, e.g., for solving a
        convex(ified) optimal control problem.
        :param x: state for linearization
        :param u: input for linearization
        :return: dynamics at (x,u) and Jacobians at (x,u) w.r.t. x and u
        """

        # convert state and input to arrays
        if isinstance(x, StateInterface):
            x_np = x.convert_to_array()
        else:
            x_np = x

        if isinstance(u, InputInterface):
            u_np = u.convert_to_array()
        else:
            u_np = u

        # evaluate discretized dynamics at (x,u)
        x_next = self._dynamics_dt(x_np, u_np).full()

        # evaluate linearized dynamics
        jac_x = self._jac_dynamics_dt_x(x_np, u_np).full()
        jac_u = self._jac_dynamics_dt_u(x_np, u_np).full()

        return x_next, jac_x, jac_u

    def _discretize(self) -> Tuple[cas.Function, cas.Function, cas.Function]:
        """
        Time-discretization of the vehicle model assuming a constant control input throughout the time interval t in
        [0, dt]
        :return: time-discretized dynamical system (CasADi function) and its Jacobians (CasADi function)
        """

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # discretize dynamics
        x_next = cas.Function(
            "dynamics_dt", [xk, uk], [rk4_integrator(xk, uk, self._dynamics_cas, self._delta_t)]
        )

        # compute Jacobian of discretized dynamics
        jac_x = cas.Function("jac_dynamics_dt", [xk, uk],
                                  [cas.jacobian(x_next(xk, uk), xk)])
        jac_u = cas.Function("jac_dynamics_dt", [xk, uk],
                                  [cas.jacobian(x_next(xk, uk), uk)])
        return x_next, jac_x, jac_u



    @abstractmethod
    def compute_normalized_acceleration(self,
                                        x: Union[StateInterface, cas.SX.sym, np.array],
                                        u: Union[InputInterface, cas.SX.sym, np.array]) \
            -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:
        pass

    def linearize_acceleration_constraints_at(self,
                               x: Union[StateInterface, np.array],
                               u: Union[InputInterface, np.array]) \
        -> Tuple[np.array, np.array, np.array, np.array, np.array, np.array]:
        """
        Linearization of the (normalized) acceleration constraint functions at a given state-input-pair, e.g., for solving
        a convex(ified) optimal control problem.
        :param x: state for linearization
        :param u: input for linearization
        :return: (normalized) acceleration constraint functions and respective Jacobians w.r.t. x and u
        """

        # convert state and input to arrays
        if isinstance(x, StateInterface):
            x_np = x.convert_to_array()
        else:
            x_np = x

        if isinstance(u, InputInterface):
            u_np = u.convert_to_array()
        else:
            u_np = u

        # evaluate acceleration constraint function at (x,u)
        a_long, a_lat = self._a_norm(x_np, u_np)
        a_long = a_long.full()
        a_lat = a_lat.full()

        # evaluate linearized constraint functions
        jac_a_long_x = self._jac_a_norm_long_x(x_np, u_np).full()
        jac_a_long_u = self._jac_a_norm_long_u(x_np, u_np).full()
        jac_a_lat_x = self._jac_a_norm_lat_x(x_np, u_np).full()
        jac_a_lat_u = self._jac_a_norm_lat_u(x_np, u_np).full()

        return a_long, a_lat, jac_a_long_x, jac_a_long_u, jac_a_lat_x, jac_a_lat_u

    def _differentiate_acceleration_constraints(self) \
            -> Tuple[cas.Function, cas.Function, cas.Function, cas.Function, cas.Function]:
        """
        Differentiation of the (normalized) acceleration constraint functions.
        :return: acceleration constraint functions (longitudinal and lateral, CasADi functions) and respective Jacobians (CasADi functions)
        """
        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # casadi function to normalized acceleration
        a_norm = cas.Function(
            "a_norm", [xk, uk], [self.compute_normalized_acceleration(xk, uk)[0], self.compute_normalized_acceleration(xk, uk)[1]],
            ['xk', 'uk'], ['a_long_norm', 'a_lat_norm']
        )

        # compute Jacobian of normalized longitudinal acceleration
        jac_a_long_x = cas.Function("jac_a_long_x", [xk, uk],
                                  [cas.jacobian(a_norm(xk, uk)[0], xk)])
        jac_a_long_u = cas.Function("jac_a_long_u", [xk, uk],
                                  [cas.jacobian(a_norm(xk, uk)[0], uk)])

        # compute Jacobian of normalized lateral acceleration
        jac_a_lat_x = cas.Function("jac_a_lat_x", [xk, uk],
                                  [cas.jacobian(a_norm(xk, uk)[1], xk)])
        jac_a_lat_u = cas.Function("jac_a_lat_u", [xk, uk],
                                  [cas.jacobian(a_norm(xk, uk)[1], uk)])

        return a_norm, jac_a_long_x, jac_a_long_u, jac_a_lat_x, jac_a_lat_u

    @property
    def state_dimension(self):
        return self._nx

    @property
    def input_dimension(self):
        return self._nu

    @abstractmethod
    def position_to_clcs(self, x: StateInterface) -> StateInterface:
        pass

    @abstractmethod
    def position_to_cartesian(self, x: StateInterface) -> StateInterface:
        pass