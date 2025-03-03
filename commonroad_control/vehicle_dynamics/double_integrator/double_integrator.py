import casadi as cas
import numpy as np
from typing import Tuple, Union
from control import StateSpace


from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices


class DoubleIntegrator(VehicleModelInterface):
    def __init__(self, params: VehicleParameters, dt: float):

        self._sys_mat, self._input_mat = self._system_matrices()

        # init base class
        super().__init__(nx=DIStateIndices.dim, nu=DIInputIndices.dim, dt=dt)

    @staticmethod
    def _system_matrices() -> Tuple[np.array, np.array]:
        """

        :return:
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

    def input_bounds(self) -> Tuple[DIInput, DIInput]:

        # lower bound
        u_lb = DIInput(acceleration_long=-9.0, acceleration_lat=-2.0)
        # upper bound
        u_ub = DIInput(acceleration_long=5.0, acceleration_lat=2.0)

        return u_lb, u_ub

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

        #
        LTI_ct = StateSpace(self._sys_mat, self._input_mat,
                            np.eye(self._nx), np.zeros((self._nx, self._nu)))
        LTI_dt = LTI_ct.sample(Ts=self._dt)
        sys_mat_dt = LTI_dt.A
        input_mat_dt = LTI_dt.B

        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)

        # discrete-time dynamics
        x_next = cas.Function("dynamics_dt", [xk, uk], [sys_mat_dt@xk + input_mat_dt@uk])

        # Jacobians of the discretized dynamics
        jac_x = cas.Function("jac_dynamics_dt_x", [xk, uk], [sys_mat_dt])
        jac_u = cas.Function("jac_dynamics_dt_u", [xk, uk], [input_mat_dt])

        return x_next, jac_x, jac_u
