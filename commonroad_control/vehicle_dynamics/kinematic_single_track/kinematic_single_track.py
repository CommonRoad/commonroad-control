from math import atan, tan, sin, cos
from typing import Type, Tuple
import numpy as np
import casadi as cas

from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class KinematicSingleStrack(VehicleModelInterface):
    def __init__(self, params: Type[VehicleParameters], dt: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r

        # init base class
        super().__init__(nx=6, nu=2, dt=dt)

    def _dynamics(self, x, u):
        " TODO: add dynaimcs function"

        v = x[2]
        a = x[3]
        psi = x[4]
        delta = x[5]

        # compute slip angle
        beta = atan(tan(delta)*self._l_r/self._l_wb)

        x_dot = v*cos(psi + beta)
        y_dot = v*sin(psi + beta)
        v_dot = a
        a_dot = u[0]
        psi_dot = v*sin(beta) / self._l_r
        delta_dot = u[1]

        f = cas.vertcat(x_dot, y_dot, v_dot, a_dot, psi_dot, delta_dot)

        return f

    def _dynamics_cas(self, x, u):
        x_next = self._dynamics(x,u)

        return cas.SX(x_next)


    def simulate_forward(self, x: StateInterface, u: InputInterface) -> StateInterface:
        pass
    def linearize(self, x: StateInterface, u: InputInterface) -> Tuple[StateInterface, np.array, np.array]:
        pass

    def position_to_clcs(self, x: StateInterface) -> StateInterface:
        pass

    def position_to_cartesian(self, x: StateInterface) -> StateInterface:
        pass

