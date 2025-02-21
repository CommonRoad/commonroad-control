from math import atan, tan, sin, cos
import numpy as np
import casadi as cas


from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from commonroad_control.vehicle_dynamics.utils import rk4_integrator


class KinematicSingleStrack(VehicleModelInterface):
    def __init__(self, params: VehicleParameters):
        # init base class
        super().__init__(nx, nu)

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r

    def _dynamics(self, x, u):
        " TODO: add dynaimcs function"

        v = x(2)
        a = x(3)
        psi = x(4)
        delta = x(5)

        # compute slip angle
        beta = atan(tan(delta)*self._l_r/self._l_wb)

        x_dot = v*cos(psi + beta)
        y_dot = v*sin(psi + beta)
        v_dot = a
        a_dot = u(1)
        psi_dot = v*sin(beta) / self._l_r
        delta_dot = u(2)

        f = [x_dot, y_dot, v_dot, a_dot, psi_dot, delta_dot]

        return f


    def discretize(self, x0: Union(np.array, cas.SX.sym), u: Union(np.array, cas.SX.sym), dt: float) -> cas.SX.sym:
        """
        Discretizes the dynamical system assuming a piece-wise constant control input.
        :param x:
        :param u:
        :param dt:
        :return:
        """

        x_next = rk4_integrator(x0 = x0, u = u, ode = self._dynamics, dt)

        return x_next
