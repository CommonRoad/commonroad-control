from typing import Type, Tuple, Union
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput, DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState, DBStateIndices
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class DynamicBicycle(VehicleModelInterface):
    def __init__(self, params: Type[VehicleParameters], dt: float):

        # set vehicle parameters
        self._m = params.m
        self._l_wb = params.l_wb
        self._l_f = params.l_f
        self._l_r = params.l_r
        self._I_zz = params.I_zz
        self._C_f = params.C_f
        self._C_r = params.C_r

        # init base class
        super().__init__(nx=DBState.dim, nu=DBInput.dim, dt=dt)

    def simulate_forward(self, x: DBState, u: DBInput) -> DBState:
        pass

    def linearize(self, x: DBState, u: DBInput) -> Tuple[DBState, np.array, np.array]:
        pass

    def position_to_clcs(self, x: DBState) -> DBState:
        pass

    def position_to_cartesian(self, x: DBState) -> DBState:
        pass

    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.array],
                      u: Union[cas.SX.sym, np.array]
                      ) -> cas.SX.sym:
        """
        Dynamics function of the dynamic bicycle model.
        Equations are taken from
        - 'R. Rajamani "Vehicle Dynamics and Control", Springer, 2011' (dynamics, p. 27)
        - 'J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", CMU-RI-TR-09-08, 2009'
        (tyre slip angles, p. 29)

        :param x: state - array of dimension (self._nx,1)
        :param u: control input - array of dimension (self._nu,1)
        :return: dynamics at (x,u) - casadi symbolic of dimension (self._nx,1)
        """

        # extract state
        v_bx = x[DBStateIndices.velocity_long]
        v_by = x[DBStateIndices.velocity_lat]
        a = x[DBStateIndices.acceleration]
        psi = x[DBStateIndices.heading]
        psi_dot = x[DBStateIndices.yaw_rate]
        delta = x[DBStateIndices.steering_angle]

        # extract control input
        j = u[DBInputIndices.jerk]
        delta_dot = u[DBInputIndices.steering_angle_velocity]

        # (tyre) slip angles
        alpha_f = cas.atan((v_by + self._l_f*psi_dot)/v_bx) - delta
        alpha_r = cas.atan((v_by - self._l_r*psi_dot)/v_bx)

        # tyre forces
        fc_f = -self._C_f*alpha_f
        fc_r = -self._C_r*alpha_r

        # dynamics
        f = cas.SX.sym('f', self._nx)

        f[DBStateIndices.position_x] = v_bx * cas.cos(psi) - v_by * cas.sin(psi)
        f[DBStateIndices.position_y] = v_bx * cas.sin(psi) + v_by * cas.cos(psi)
        f[DBStateIndices.velocity_long] = psi_dot * v_by + a
        f[DBStateIndices.velocity_lat] = -psi_dot * v_bx + (fc_f * cas.cos(delta) + fc_r) * 2 / self._m
        f[DBStateIndices.acceleration] = j
        f[DBStateIndices.heading] = psi_dot
        f[DBStateIndices.yaw_rate] = (self._l_f * fc_f - self._l_r * fc_r) * 2 / self._I_zz
        f[DBStateIndices.steering_angle] = delta_dot

        return f
