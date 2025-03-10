import casadi as cas
import numpy as np
from typing import Tuple, Union
import scipy.signal as scsi


from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory


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
        """

        :return:
        """

        # lower bound
        u_lb = DIInput(acceleration_long=-9.0, acceleration_lat=-2.0)
        # upper bound
        u_ub = DIInput(acceleration_long=5.0, acceleration_lat=2.0)

        return u_lb, u_ub

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

        # compute matrices of discrete-time LTI system
        lit_ct = scsi.lti(self._sys_mat, self._input_mat,
                            np.eye(self._nx), np.zeros((self._nx, self._nu)))
        lit_dt = lit_ct.to_discrete(dt=self._dt, method='zoh')
        sys_mat_dt = lit_dt.A
        input_mat_dt = lit_dt.B

        # discrete-time dynamics
        xk = cas.SX.sym("xk", self._nx, 1)
        uk = cas.SX.sym("uk", self._nu, 1)
        x_next = cas.Function("dynamics_dt", [xk, uk], [sys_mat_dt@xk + input_mat_dt@uk])

        # Jacobians of the discrete-time dynamics
        jac_x = cas.Function("jac_dynamics_dt_x", [xk, uk], [sys_mat_dt])
        jac_u = cas.Function("jac_dynamics_dt_u", [xk, uk], [input_mat_dt])

        return x_next, jac_x, jac_u
