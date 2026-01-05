from abc import ABC, abstractmethod
from typing import Any, Tuple, Union

import casadi as cas
import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.utils import rk4_integrator
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class VehicleModelInterface(ABC):
    """
    Interface for vehicle dynamics models: among others, the classes implementing this interface provide the dynamics function, both in continous- and discrete-time, or methods for computing the longitudinal and lateral accelerations.
    """

    @classmethod
    @abstractmethod
    def factory_method(cls, params: VehicleParameters, delta_t: float) -> Any:
        """
        Factory method to generate class
        :param params: CommonRoad-Control vehicle parameters
        :param delta_t: sampling time
        :return: instance
        """
        pass

    def __init__(self, params: VehicleParameters, nx: int, nu: int, nw: int, delta_t: float):
        """
        Initialize abstract baseclass.
        :param params: CommonRoad-Control vehicle parameters
        :param nx: dimension of the state space
        :param nu: dimension of the input space
        :param nw: dimension of the disturbance space
        :param delta_t: sampling time
        """
        self._nx: int = nx
        self._nu: int = nu
        self._nw: int = nw
        self._delta_t: float = delta_t

        # input bounds
        self._u_lb, self._u_ub = self._set_input_bounds(params)

        # discretize (nominal) vehicle model
        self._dynamics_dt, self._jac_dynamics_dt_x, self._jac_dynamics_dt_u = self._discretize_nominal()

        # differentiate acceleration constraint functions
        (
            self._a_norm,
            self._jac_a_norm_long_x,
            self._jac_a_norm_long_u,
            self._jac_a_norm_lat_x,
            self._jac_a_norm_lat_u,
        ) = self._differentiate_acceleration_constraints()

    def simulate_dt_nom(
        self,
        x: Union[StateInterface, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[InputInterface, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> np.ndarray:
        """
        One-step simulation of the time-discretized nominal vehicle dynamics.
        :param x: initial state - StateInterface/ array of dimension (self._nx,)
        :param u: control input - InputInterface/ array of dimension (self._nu,)
        :return: nominal state at next time step
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

        return x_next.squeeze()

    def dynamics_ct(
        self,
        x: Union[StateInterface, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[InputInterface, np.ndarray[tuple[float], np.dtype[np.float64]]],
        w: Union[DisturbanceInterface, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> np.ndarray:
        """
        Interface to the continuous-time dynamics function of the vehicle model.
        :param x: state - StateInterface/ array of dimension (self._nx,)
        :param u: control input - InputInterface/ array of dimension (self._nu,)
        :param w: disturbance - DisturbanceInterface/ array of dimension (self._nw,)
        :return: dynamics function evaluated at (x, u, w) - array of dimension (self._nx,)
        """

        # convert state, input, and disturbance to arrays
        if isinstance(x, StateInterface):
            x_np = x.convert_to_array()
        else:
            x_np = x

        if isinstance(u, InputInterface):
            u_np = u.convert_to_array()
        else:
            u_np = u

        if isinstance(w, DisturbanceInterface):
            w_np = w.convert_to_array()
        else:
            w_np = w

        x_next = self._dynamics_cas(x_np, u_np, w_np)
        x_next = np.reshape(x_next, (1, self._nx), order="F").squeeze()

        return x_next

    @abstractmethod
    def _dynamics_cas(
        self,
        x: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        w: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> Union[cas.SX.sym, np.ndarray]:
        """
        Continuous-time dynamics function of the vehicle model for simulation and symbolic operations using CasADi.
        :param x: state - CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - CasADi symbolic/ array of dimension (self._nu,)
        :param w: disturbance - CasADi symbolic/ array of dimension (self._nw,)
        :return: dynamics function evaluated at (x,u,w) - CasADi symbolic/ array of dimension (self._nx,)
        """
        pass

    def linearize_dt_nom_at(
        self, x: Union[StateInterface, np.array], u: Union[InputInterface, np.array]
    ) -> Tuple[np.array, np.array, np.array]:
        """
        Linearization of the time-discretized nominal vehicle dynamics at a given state-input-pair, e.g., for solving a
        convex(ified) optimal control problem.
        :param x: state for linearization - StateInterface/ array of dimension (self._nx,)
        :param u: input for linearization - InputInterface/ array of dimension (self._nu,)
        :return: nominal dynamics at (x,u) and Jacobians at (x,u) w.r.t. x and u
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

    def _discretize_nominal(self) -> Tuple[cas.Function, cas.Function, cas.Function]:
        """
        Time-discretization of the nominal dynamics model assuming a constant control input throughout the time interval [0, delta_t].
        :return: time-discretized dynamical system (CasADi function) and its Jacobians (CasADi function)
        """

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)
        # nominal disturbance
        wk = np.zeros((self._nw,))

        # discretize dynamics
        x_next = cas.Function(
            "dynamics_dt",
            [xk, uk],
            [rk4_integrator(xk, uk, wk, self._dynamics_cas, self._delta_t)],
        )

        # compute Jacobian of discretized dynamics
        jac_x = cas.Function("jac_dynamics_dt", [xk, uk], [cas.jacobian(x_next(xk, uk), xk)])
        jac_u = cas.Function("jac_dynamics_dt", [xk, uk], [cas.jacobian(x_next(xk, uk), uk)])
        return x_next, jac_x, jac_u

    @abstractmethod
    def compute_normalized_acceleration(
        self,
        x: Union[StateInterface, cas.SX.sym, np.array],
        u: Union[InputInterface, cas.SX.sym, np.array],
    ) -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:
        """
        Computes the normalized longitudinal and lateral acceleration (w.r.t. the maximum acceleration).
        :param x: state - StateInterface/ CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - InputInterface/ CasADi symbolic/ array of dimension (self._nu,)
        :return: normalized longitudinal and lateral acceleration - float/ CasADi symbolic
        """
        pass

    def linearize_acceleration_constraints_at(
        self, x: Union[StateInterface, np.array], u: Union[InputInterface, np.array]
    ) -> Tuple[np.array, np.array, np.array, np.array, np.array, np.array]:
        """
        Linearization of the (normalized) acceleration constraint functions at a given state-input-pair, e.g., for solving
        a convex(ified) optimal control problem.
        :param x: state for linearization - array of dimension (self._nx,)
        :param u: input for linearization - array of dimension (self._nu,)
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

    def _differentiate_acceleration_constraints(
        self,
    ) -> Tuple[cas.Function, cas.Function, cas.Function, cas.Function, cas.Function]:
        """
        Differentiation of the (normalized) acceleration constraint functions.
        :return: acceleration constraint functions (longitudinal and lateral, CasADi functions) and respective Jacobians (CasADi functions)
        """
        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # casadi function to normalized acceleration
        a_norm = cas.Function(
            "a_norm",
            [xk, uk],
            [
                self.compute_normalized_acceleration(xk, uk)[0],
                self.compute_normalized_acceleration(xk, uk)[1],
            ],
            ["xk", "uk"],
            ["a_long_norm", "a_lat_norm"],
        )

        # compute Jacobian of normalized longitudinal acceleration
        jac_a_long_x = cas.Function("jac_a_long_x", [xk, uk], [cas.jacobian(a_norm(xk, uk)[0], xk)])
        jac_a_long_u = cas.Function("jac_a_long_u", [xk, uk], [cas.jacobian(a_norm(xk, uk)[0], uk)])

        # compute Jacobian of normalized lateral acceleration
        jac_a_lat_x = cas.Function("jac_a_lat_x", [xk, uk], [cas.jacobian(a_norm(xk, uk)[1], xk)])
        jac_a_lat_u = cas.Function("jac_a_lat_u", [xk, uk], [cas.jacobian(a_norm(xk, uk)[1], uk)])

        return a_norm, jac_a_long_x, jac_a_long_u, jac_a_lat_x, jac_a_lat_u

    @abstractmethod
    def _set_input_bounds(self, params: VehicleParameters):
        """
        Extract input bounds from vehicle parameters and returns them as instances of the Input class.
        :param params: CommonRoad-Control vehicle parameters
        :return: lower and upper bounds - InputInterface
        """
        pass

    def input_bounds(self) -> Tuple[InputInterface, InputInterface]:
        """
        Returns the lower and upper bound on the control inputs.
        :return: lower bound and upper bound - InputInterface
        """
        return self._u_lb, self._u_ub

    @property
    def state_dimension(self):
        """
        :return: dimension of the state space
        """
        return self._nx

    @property
    def input_dimension(self):
        """
        :return: dimension of the input space
        """
        return self._nu

    @property
    def disturbance_dimension(self):
        """
        :return: dimension of the disturbance space
        """
        return self._nw
