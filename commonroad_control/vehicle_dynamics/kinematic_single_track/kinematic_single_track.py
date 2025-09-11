from typing import Type, Tuple, Union
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput, KSTInputIndices
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState, KSTStateIndices
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class KinematicSingleStrack(VehicleModelInterface):
    def __init__(self, params: VehicleParameters, delta_t: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r
        self._a_long_max = params.a_long_max
        self._a_lat_max = params.a_lat_max

        # init base class
        super().__init__(
            params=params,
            nx=KSTState.dim,
            nu=KSTInput.dim,
            delta_t=delta_t
        )

    def simulate_forward(self, x: KSTState, u: KSTInput) -> KSTState:
        pass

    def linearize(self, x: KSTState, u: KSTInput) -> Tuple[KSTState, np.array, np.array]:
        pass

    def position_to_clcs(self, x: KSTState) -> KSTState:
        pass

    def position_to_cartesian(self, x: KSTState) -> KSTState:
        pass

    def _set_input_bounds(self,
                          params: VehicleParameters) \
        -> Tuple[KSTInput, KSTInput]:
        """
        Extract input bounds from vehicle parameters and store as instance of InputInterface class.
        :param params: vehicle parameters
        :return: lower and upper bound on the inputs
        """

        # lower bound
        u_lb = KSTInput(
            acceleration=-params.a_long_max,
            steering_angle_velocity=-params.steering_angle_velocity_max
        )

        # upper bound
        u_ub = KSTInput(
            acceleration=params.a_long_max,
            steering_angle_velocity=params.steering_angle_velocity_max
        )

        return u_lb, u_ub

    def _dynamics_cas_clcs(self,
                           x: Union[cas.SX.sym, np.array],
                           u: Union[cas.SX.sym, np.array],
                           p: Union[cas.SX.sym, np.array]):
        """

        :param x:
        :param u:
        :param p: reference parameters, e.g.
        :return:
        """

        # compute cartesian dynamics
        f = self._dynamics_cas(x, u)

        # extract state
        pos_y = x[KSTStateIndices.position_y]
        v = x[KSTStateIndices.velocity]
        psi = x[KSTStateIndices.heading]
        delta = x[KSTStateIndices.steering_angle]

        # extract parameters
        kappa_ref = p.curvature

        # compute slip angle
        beta = cas.atan(cas.tan(delta) * self._l_r / self._l_wb)

        # dynamics
        f[KSTStateIndices.position_x] = v*cas.cos(psi-psi_ref)/(1 - pos_y*kappa_ref)
        f[KSTStateIndices.position_y] = v * cas.sin(psi - psi_ref)

        return f

    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.array],
                      u: Union[cas.SX.sym, np.array]) \
            -> cas.SX.sym:
        """
         Dynamics function of the kinematic single-track model.

        :param x: state - array of dimension (self._nx,1)
        :param u: control input - array of dimension (self._nu,1)
        :return: dynamics at (x,u) - casadi symbolic of dimension (self._nx,1)
        """

        # extract state
        v = x[KSTStateIndices.velocity]
        psi = x[KSTStateIndices.heading]
        delta = x[KSTStateIndices.steering_angle]

        # extract control input
        a = u[KSTInputIndices.acceleration]
        delta_dot = u[KSTInputIndices.steering_angle_velocity]

        # compute slip angle
        beta = cas.atan(cas.tan(delta)*self._l_r/self._l_wb)

        # dynamics
        position_x_dot = v*cas.cos(psi + beta)
        position_y_dot = v*cas.sin(psi + beta)
        velocity_dot = a
        heading_dot = v*cas.sin(beta) / self._l_r
        steering_angle_dot = delta_dot

        f = cas.vertcat(position_x_dot, position_y_dot, velocity_dot,
                        heading_dot, steering_angle_dot)

        return f

    def compute_normalized_acceleration(self,
                                        x: Union[KSTState, cas.SX.sym, np.array],
                                        u: Union[KSTInput, cas.SX.sym, np.array]) \
        -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:

        # extract state
        if isinstance(x, KSTState):
            x = x.convert_to_array()
        v = x[KSTStateIndices.velocity]
        delta = x[KSTStateIndices.steering_angle]

        # compute slip angle
        beta = cas.atan(cas.tan(delta)*self._l_r/self._l_wb)
        # compute yaw rate
        heading_dot = v*cas.sin(beta) / self._l_r

        # extract control input
        if isinstance(u, KSTInput):
            u = u.convert_to_array()
        a = u[KSTInputIndices.acceleration]

        # normalized acceleration
        a_long_norm = a / self._a_long_max
        a_lat_norm = (v*heading_dot) / self._a_lat_max

        return a_long_norm, a_lat_norm
