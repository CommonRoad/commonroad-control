from typing import Type, Tuple, Union
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class KinematicSingleStrack(VehicleModelInterface):
    def __init__(self, params: Type[VehicleParameters], dt: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r

        # init base class
        super().__init__(nx=KSTState.dim, nu=KSTInput.dim, dt=dt)

    def _dynamics_cas(self,
                      x: Union[cas.SX.sym, np.array],
                      u: Union[cas.SX.sym, np.array]) \
            -> cas.SX.sym:

        v = x[2]
        a = x[3]
        psi = x[4]
        delta = x[5]

        # compute slip angle
        beta = cas.atan(cas.tan(delta)*self._l_r/self._l_wb)

        x_dot = v*cas.cos(psi + beta)
        y_dot = v*cas.sin(psi + beta)
        v_dot = a
        a_dot = u[0]
        psi_dot = v*cas.sin(beta) / self._l_r
        delta_dot = u[1]

        f = cas.vertcat(x_dot, y_dot, v_dot, a_dot, psi_dot, delta_dot)

        return f

    def simulate_forward(self, x: KSTState, u: KSTInput) -> KSTState:
        pass

    def linearize(self, x: KSTState, u: KSTInput) -> Tuple[KSTState, np.array, np.array]:
        pass

    def position_to_clcs(self, x: KSTState) -> KSTState:
        pass

    def position_to_cartesian(self, x: KSTState) -> KSTState:
        pass
