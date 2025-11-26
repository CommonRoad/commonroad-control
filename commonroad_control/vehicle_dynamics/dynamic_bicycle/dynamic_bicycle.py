from typing import Tuple, Union

import casadi as cas
import numpy as np

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_disturbance import (
    DBDisturbanceIndices,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import (
    DBInput,
    DBInputIndices,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import (
    DBState,
    DBStateIndices,
)
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class DynamicBicycle(VehicleModelInterface):

    @classmethod
    def factory_method(
        cls, params: VehicleParameters, delta_t: float
    ) -> "DynamicBicycle":
        """
        Factory method to generate class
        :param params: CommonRoad vehicle params
        :param delta_t: sampling time
        :return: instance
        """
        return DynamicBicycle(params=params, delta_t=delta_t)

    def __init__(self, params: VehicleParameters, delta_t: float):

        # set vehicle parameters
        self._g = params.g
        self._m = params.m
        self._l_wb = params.l_wb
        self._l_f = params.l_f
        self._l_r = params.l_r
        self._I_zz = params.I_zz
        self._C_f = params.C_f
        self._C_r = params.C_r
        self._h_cog = params.h_cog
        self._a_long_max = params.a_long_max
        self._a_lat_max = params.a_lat_max

        # init base class
        super().__init__(
            params=params,
            nx=DBStateIndices.dim,
            nu=DBInputIndices.dim,
            nw=DBDisturbanceIndices.dim,
            delta_t=delta_t,
        )

    def position_to_clcs(self, x: DBState) -> DBState:
        pass

    def position_to_cartesian(self, x: DBState) -> DBState:
        pass

    def _set_input_bounds(self, params: VehicleParameters) -> Tuple[DBInput, DBInput]:
        """
        Extract input bounds from vehicle parameters and store as instance of InputInterface class.
        :param params: vehicle parameters
        :return: lower and upper bound on the inputs
        """

        # lower bound
        u_lb = DBInput(
            acceleration=-params.a_long_max,
            steering_angle_velocity=-params.steering_angle_velocity_max,
        )

        # upper bound
        u_ub = DBInput(
            acceleration=params.a_long_max,
            steering_angle_velocity=params.steering_angle_velocity_max,
        )

        return u_lb, u_ub

    def _dynamics_cas(
        self,
        x: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        w: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> cas.SX.sym:
        """
        Dynamics function of the dynamic bicycle model.
        Equations are taken from
        - 'J. Kabzan et al. "Learning-Based Model Predictive Control for Autonomous Racing", IEEE RA-L, 2019'
        - 'J. M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", CMU-RI-TR-09-08, 2009'
        (tyre slip angles, p. 29)

        :param x: state - array of dimension (self._nx,1)
        :param u: control input - array of dimension (self._nu,1)
        :return: dynamics at (x,u) - casadi symbolic of dimension (self._nx,1)
        """

        # extract state
        v_bx = x[DBStateIndices.velocity_long]
        v_by = x[DBStateIndices.velocity_lat]
        psi = x[DBStateIndices.heading]
        psi_dot = x[DBStateIndices.yaw_rate]
        delta = x[DBStateIndices.steering_angle]

        # extract control input
        a = u[DBInputIndices.acceleration]
        delta_dot = u[DBInputIndices.steering_angle_velocity]

        # compute lateral tyre forces
        fc_f, fc_r = self._compute_lateral_tyre_forces(x, u)

        # dynamics
        position_x_dot = v_bx * cas.cos(psi) - v_by * cas.sin(psi)
        position_y_dot = v_bx * cas.sin(psi) + v_by * cas.cos(psi)
        velocity_long_dot = psi_dot * v_by + a - (fc_f * cas.sin(delta)) / self._m
        velocity_lat_dot = -psi_dot * v_bx + (fc_f * cas.cos(delta) + fc_r) / self._m
        heading_dot = psi_dot
        yaw_rate_dot = (
            self._l_f * fc_f * cas.cos(delta) - self._l_r * fc_r
        ) / self._I_zz
        steering_angle_dot = delta_dot

        f = cas.vertcat(
            position_x_dot,
            position_y_dot,
            velocity_long_dot,
            velocity_lat_dot,
            heading_dot,
            yaw_rate_dot,
            steering_angle_dot,
        )

        # add disturbances
        f = f + np.reshape(w, shape=(w.size, 1))

        return f

    def compute_normalized_acceleration(
        self,
        x: Union[DBState, cas.SX.sym, np.array],
        u: Union[DBInput, cas.SX.sym, np.array],
    ) -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:

        # extract state
        if isinstance(x, DBState):
            x = x.convert_to_array()
        delta = x[DBStateIndices.steering_angle]

        # extract control input
        if isinstance(u, DBInput):
            u = u.convert_to_array()
        a = u[DBInputIndices.acceleration]

        # compute lateral tyre forces
        fc_f, fc_r = self._compute_lateral_tyre_forces(x, u)

        # normalized acceleration
        a_long_norm = (a - fc_f * cas.sin(delta) / self._m) / self._a_long_max
        a_lat_norm = ((fc_f * cas.cos(delta) + fc_r) / self._m) / self._a_lat_max

        return a_long_norm, a_lat_norm

    def _compute_lateral_tyre_forces(
        self, x: Union[cas.SX.sym, np.array], u: Union[cas.SX.sym, np.array]
    ) -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:
        """
        Computes the lateral tyre forces at the front and rear axle.
        :param x: state - array of dimension (self._nx,)
        :param u: control input - array of dimension (self._nu,)
        :return: lateral tyre forces at front and rear axle
        """
        # extract state
        v_bx = x[DBStateIndices.velocity_long]
        v_by = x[DBStateIndices.velocity_lat]
        psi_dot = x[DBStateIndices.yaw_rate]
        delta = x[DBStateIndices.steering_angle]

        # extract control input
        a = u[DBInputIndices.acceleration]

        # (tyre) slip angles
        alpha_f = cas.atan((v_by + self._l_f * psi_dot) / v_bx) - delta
        alpha_r = cas.atan((v_by - self._l_r * psi_dot) / v_bx)

        # compute normal forces per axle (including longitudinal load transfer)
        fz_f = (self._m * self._g * self._l_r - self._m * a * self._h_cog) / self._l_wb
        fz_r = (self._m * self._g * self._l_f + self._m * a * self._h_cog) / self._l_wb

        # lateral tyre forces per axle
        fc_f = -self._C_f * alpha_f * fz_f
        fc_r = -self._C_r * alpha_r * fz_r

        return fc_f, fc_r
