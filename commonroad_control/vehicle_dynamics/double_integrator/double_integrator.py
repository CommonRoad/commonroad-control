from typing import Tuple, Union

import casadi as cas
import numpy as np
import scipy.signal as scsi

from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import (
    DIDisturbanceIndices,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_input import (
    DIInput,
    DIInputIndices,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_state import (
    DIState,
    DIStateIndices,
)
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class DoubleIntegrator(VehicleModelInterface):
    """
    Double integrator model.
    """

    @classmethod
    def factory_method(
        cls, params: VehicleParameters, delta_t: float
    ) -> "DoubleIntegrator":
        """
        Factory method to generate class
        :param params: CommonRoad-Control vehicle params
        :param delta_t: sampling time
        :return: instance
        """
        return DoubleIntegrator(params=params, delta_t=delta_t)

    def __init__(self, params: VehicleParameters, delta_t: float):

        self._sys_mat, self._input_mat = self._system_matrices()

        # set vehicle parameters
        self._a_long_max = params.a_long_max
        self._a_lat_max = params.a_lat_max

        # init base class
        super().__init__(
            params=params,
            nx=DIStateIndices.dim,
            nu=DIInputIndices.dim,
            nw=DIDisturbanceIndices.dim,
            delta_t=delta_t,
        )

    @staticmethod
    def _system_matrices() -> Tuple[np.array, np.array]:
        """
        :return: system and input matrix - arrays of dimension (self._nx, self._nx) and (self._nx, self._nu)
        """

        # system matrix
        sys_mat = np.zeros((DIStateIndices.dim, DIStateIndices.dim))
        sys_mat[DIStateIndices.position_long, DIStateIndices.velocity_long] = 1.0
        sys_mat[DIStateIndices.position_lat, DIStateIndices.velocity_lat] = 1.0

        # input matrix
        input_mat = np.zeros((DIStateIndices.dim, DIInputIndices.dim))
        input_mat[DIStateIndices.velocity_long, DIInputIndices.acceleration_long] = 1.0
        input_mat[DIStateIndices.velocity_lat, DIInputIndices.acceleration_lat] = 1.0

        return sys_mat, input_mat

    def _set_input_bounds(self, params: VehicleParameters) -> Tuple[DIInput, DIInput]:
        """
        Extract input bounds from vehicle parameters and returns them as instances of the Input class.
        :param params: vehicle parameters
        :return: lower and upper bound on the inputs - DIInputs
        """

        # lower bound
        u_lb = DIInput(
            acceleration_long=-params.a_long_max, acceleration_lat=-params.a_lat_max
        )
        # upper bound
        u_ub = DIInput(
            acceleration_long=params.a_long_max, acceleration_lat=params.a_lat_max
        )

        return u_lb, u_ub

    def _dynamics_cas(
        self,
        x: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        w: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> cas.SX.sym:
        """
        Continuous-time dynamics function of the vehicle model for simulation and symbolic operations using CasADi.
        :param x: state - CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - CasADi symbolic/ array of dimension (self._nu,)
        :param w: disturbance - CasADi symbolic/ array of dimension (self._nw,)
        :return: dynamics function evaluated at (x,u,w) - CasADi symbolic/ array of dimension (self._nx,)
        """

        return self._sys_mat @ x + self._input_mat @ u + w

    @staticmethod
    def state_bounds(self) -> Tuple[np.array, np.array, np.array, np.array]:
        """

        :return:
        """
        # lower bound lb <= mat_lb*x
        mat_lb = np.zeros((2, DIStateIndices.dim), dtype=float)
        mat_lb[0, DIStateIndices.velocity_long] = 1.0
        mat_lb[1, DIStateIndices.velocity_lat] = 1.0

        lb = np.zeros((DIStateIndices.dim, 1), dtype=float)
        lb[DIStateIndices.velocity_long] = 0.0
        lb[DIStateIndices.velocity_lat] = -2.0
        lb = mat_lb @ lb

        # uppber bound mat_ub*x <= ub
        mat_ub = mat_lb

        ub = np.zeros((DIStateIndices.dim, 1), dtype=float)
        ub[DIStateIndices.velocity_long] = 10.0
        ub[DIStateIndices.velocity_lat] = 2.0
        ub = mat_ub @ ub

        return mat_lb, lb, mat_ub, ub

    def _discretize_nominal(self) -> Tuple[cas.Function, cas.Function, cas.Function]:
        """
        Time-discretization of the nominal dynamics model assuming a constant control input throughout the time interval [0, dt].
        :return: time-discretized dynamical system (CasADi function) and its Jacobians (CasADi function)
        """

        # compute matrices of discrete-time LTI system
        lit_ct = scsi.lti(
            self._sys_mat,
            self._input_mat,
            np.eye(self._nx),
            np.zeros((self._nx, self._nu)),
        )
        lit_dt = lit_ct.to_discrete(dt=self._delta_t, method="zoh")
        sys_mat_dt = lit_dt.A
        input_mat_dt = lit_dt.B

        # discrete-time dynamics
        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)
        x_next = cas.Function(
            "dynamics_dt", [xk, uk], [sys_mat_dt @ xk + input_mat_dt @ uk]
        )

        # Jacobians of the discrete-time dynamics
        jac_x = cas.Function("jac_dynamics_dt_x", [xk, uk], [sys_mat_dt])
        jac_u = cas.Function("jac_dynamics_dt_u", [xk, uk], [input_mat_dt])

        return x_next, jac_x, jac_u

    def compute_normalized_acceleration(
        self,
        x: Union[DIState, cas.SX.sym, np.array],
        u: Union[DIInput, cas.SX.sym, np.array],
    ) -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:
        """
        Computes the normalized longitudinal and lateral acceleration (w.r.t. the maximum acceleration).
        :param x: state - State/ CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - Input/ CasADi symbolic/ array of dimension (self._nu,)
        :return: normalized longitudinal and lateral acceleration - float/ CasADi symbolic
        """

        # extract control input
        if isinstance(u, DIInput):
            u = u.convert_to_array()
        a_long = u[DIInputIndices.acceleration_long]
        a_lat = u[DIInputIndices.acceleration_lat]

        # normalized acceleration
        a_long_norm = a_long / self._a_long_max
        a_lat_norm = a_lat / self._a_lat_max

        return a_long_norm, a_lat_norm
