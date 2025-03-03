import casadi as cas
import numpy as np
import scipy as sp
from typing import Tuple, Union

from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices


class DoubleIntegrator(VehicleModelInterface):
    def __init__(self, params: VehicleParameters, dt: float):

        # continuous-time dynamics matrices
        self._sys_mat = np.array([[0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 0, 0], [0, 0, 0, 0]], dtype=float)
        self._input_mat = np.array([[0, 0], [0, 0], [1, 0], [0, 1]], dtype=float)

        # discrete-time dynamics matrices
        self._sys_mat_dt = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)
        self._input_mat_dt = np.array([[0.5*dt**2, 0], [0, 0.5*dt**2], [dt, 0], [0, dt]], dtype=float)

        # init base class
        super().__init__(nx=DIStateIndices.dim, nu=DIInputIndices.dim, dt=dt)

    def simulate_forward(self, x: DIState, u: DIInput) -> DIState:
        pass

    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.array],
                      u: Union[cas.SX.sym, np.array]) -> cas.SX.sym:
        """

        :param x:
        :param u:
        :return:
        """

        return self._sys_mat@x + self._input_mat@u

    def linearize(self, x: DIState, u: DIInput) -> Tuple[DIState, np.array, np.array]:
        pass

    def position_to_clcs(self, x: DIState) -> DIState:
        pass

    def position_to_cartesian(self, x: DIState) -> DIState:
        pass

    def _discretize(self) -> Tuple[cas.Function, cas.Function, cas.Function]:
        """

        :return:
        """

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # discrete-time dynamics
        x_next = cas.Function("dynamics_dt", [xk, uk], [self._sys_mat_dt@xk + self._input_mat_dt@uk])

        # Jacobians of the discretized dynamics
        jac_x = cas.Function("jac_dynamics_dt_x", [xk, uk], [self._sys_mat_dt])
        jac_u = cas.Function("jac_dynamics_dt_u", [xk, uk], [self._input_mat_dt])

        return x_next, jac_x, jac_u
