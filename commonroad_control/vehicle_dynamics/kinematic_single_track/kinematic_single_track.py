from typing import Type, Tuple, Union
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput, KSTInputIndices
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState, KSTStateIndices
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class KinematicSingleStrack(VehicleModelInterface):
    def __init__(self, params: VehicleParameters, dt: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r

        # init base class
        super().__init__(nx=KSTState.dim, nu=KSTInput.dim, dt=dt)

    def simulate_forward(self, x: KSTState, u: KSTInput) -> KSTState:
        pass

    def linearize(self, x: KSTState, u: KSTInput) -> Tuple[KSTState, np.array, np.array]:
        pass

    def position_to_clcs(self, x: KSTState) -> KSTState:
        pass

    def position_to_cartesian(self, x: KSTState) -> KSTState:
        pass

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
