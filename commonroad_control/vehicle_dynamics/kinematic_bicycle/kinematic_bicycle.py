from typing import Type, Tuple, Union
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput, KBInputIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState, KBStateIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_disturbance import KBDisturbanceIndices


class KinematicBicycle(VehicleModelInterface):

    @classmethod
    def factory_method(
            cls,
            params: VehicleParameters,
            delta_t: float
    ) -> "KinematicBicycle":
        """
        Factory method to generate class
        :param params: CommonRoad vehicle params
        :param delta_t: sampling time
        :return: instance
        """
        return KinematicBicycle(params=params, delta_t=delta_t)

    def __init__(self, params: VehicleParameters, delta_t: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r
        self._a_long_max = params.a_long_max
        self._a_lat_max = params.a_lat_max

        # init base class
        super().__init__(
            params=params,
            nx=KBStateIndices.dim,
            nu=KBInputIndices.dim,
            nw=KBDisturbanceIndices.dim,
            delta_t=delta_t
        )

    def position_to_clcs(self, x: KBState) -> KBState:
        pass

    def position_to_cartesian(self, x: KBState) -> KBState:
        pass

    def _set_input_bounds(self,
                          params: VehicleParameters) \
        -> Tuple[KBInput, KBInput]:
        """
        Extract input bounds from vehicle parameters and store as instance of InputInterface class.
        :param params: vehicle parameters
        :return: lower and upper bound on the inputs
        """

        # lower bound
        u_lb = KBInput(
            acceleration=-params.a_long_max,
            steering_angle_velocity=-params.steering_angle_velocity_max
        )

        # upper bound
        u_ub = KBInput(
            acceleration=params.a_long_max,
            steering_angle_velocity=params.steering_angle_velocity_max
        )

        return u_lb, u_ub

    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
                      u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
                      w: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]]) \
            -> cas.SX.sym:
        """
        Dynamics function of the kinematic bicycle model.
        We model the movement of the center of gravity of the vehicle.
        :param x: state - array of dimension (self._nx,)
        :param u: control input - - array of dimension (self._nu,)
        :param w: disturbance - - array of dimension (self._nw,)
        :return: dynamics at (x,u,w) - casadi symbolic of dimension (self._nx,1)
        """

        # extract state
        v = x[KBStateIndices.velocity]
        psi = x[KBStateIndices.heading]
        delta = x[KBStateIndices.steering_angle]

        # extract control input
        a = u[KBInputIndices.acceleration]
        delta_dot = u[KBInputIndices.steering_angle_velocity]

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

        # add disturbances
        f = f + np.reshape(w, shape=(w.size,1))

        return f

    def compute_normalized_acceleration(self,
                                        x: Union[KBState, cas.SX.sym, np.array],
                                        u: Union[KBInput, cas.SX.sym, np.array]) \
        -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:

        # extract state
        if isinstance(x, KBState):
            x = x.convert_to_array()
        v = x[KBStateIndices.velocity]
        delta = x[KBStateIndices.steering_angle]

        # compute slip angle
        beta = cas.atan(cas.tan(delta)*self._l_r/self._l_wb)
        # compute yaw rate
        heading_dot = v*cas.sin(beta) / self._l_r

        # extract control input
        if isinstance(u, KBInput):
            u = u.convert_to_array()
        a = u[KBInputIndices.acceleration]

        # normalized acceleration
        a_long_norm = a / self._a_long_max
        a_lat_norm = (v*heading_dot) / self._a_lat_max

        return a_long_norm, a_lat_norm
