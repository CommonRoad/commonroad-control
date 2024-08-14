from typing import Union, Dict, List
import numpy as np
from math import inf, pi
from optimalControl.ocp_dataclasses import State, ControlInput
import casadi as cas
from dataclasses import dataclass


@dataclass
class StateConstrainedIntegrator(State):
    dim: int = 6
    velocity_x: float = None
    velocity_y: float = None
    velocity_orientation: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim, position_x=self.position_x, position_y=self.position_y,
                         orientation=self.orientation)

    def convert_to_array(self) -> np.array:
        x_np = np.zeros((self.dim, 1), dtype=float)
        x_np[0] = self.position_x
        x_np[1] = self.position_y
        x_np[2] = self.orientation
        x_np[3] = self.velocity_x
        x_np[4] = self.velocity_y
        x_np[5] = self.velocity_orientation

        return x_np

    def set_values_from_array(self, x_np: np.array):
        self.position_x = x_np[0]
        self.position_y = x_np[1]
        self.orientation = x_np[2]
        self.velocity_x = x_np[3]
        self.velocity_y = x_np[4]
        self.velocity_orientation = x_np[5]


@dataclass
class ControlInputConstrainedIntegrator(ControlInput):
    dim: int = 3
    acceleration_x: float = None
    acceleration_y: float = None
    acceleration_orientation: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.array:
        u_np = np.zeros((self.dim, 1), dtype=float)
        u_np[0] = self.acceleration_x
        u_np[1] = self.acceleration_y
        u_np[2] = self.acceleration_orientation

        return u_np

    def set_values_from_array(self, u_np: np.array):
        self.acceleration_x = u_np[0]
        self.acceleration_y = u_np[1]
        self.acceleration_orientation = u_np[2]


def get_params() -> Dict:

    params = dict()

    params['system'] = 'ConstrainedIntegrator'

    params['N'] = 10                    # (discrete) prediction horizon
    params['dt'] = 0.1                  # sampling time
    params['nx'] = 6                    # state dimension
    params['nu'] = 3                    # control input dimension

    params['n_ineq_con_ca'] = 0

    params['penaltyWeight'] = 1000      # weight of soft constraint violation penalty

    # state and input bounds
    params['lbx'] = StateConstrainedIntegrator(position_x=-inf, position_y=-inf, orientation=-inf,
                                               velocity_x=0.0, velocity_y=-6.0, velocity_orientation=-pi)
    params['ubx'] = StateConstrainedIntegrator(position_x=inf, position_y=inf, orientation=inf,
                                               velocity_x=25.0, velocity_y=6.0, velocity_orientation=pi)
    params['lbu'] = ControlInputConstrainedIntegrator(acceleration_x=-9.0, acceleration_y=-12,
                                                      acceleration_orientation=-12)
    params['ubu'] = ControlInputConstrainedIntegrator(acceleration_x=4.0, acceleration_y=12,
                                                      acceleration_orientation=12)

    # initial state
    position_x = 0
    position_y = 0
    orientation = 0
    velocity_x = 15
    velocity_y = 0
    velocity_orientation = 0
    params['x0'] = StateConstrainedIntegrator(position_x=position_x, position_y=position_y, orientation=orientation,
                                              velocity_x=velocity_x, velocity_y=velocity_y,
                                              velocity_orientation=velocity_orientation)

    return params


def dynamics(x: Union[cas.SX.sym, np.array], u: Union[cas.SX.sym, np.array]) -> cas.SX.sym:
    """
    Flow function of the ordinary differential governing the evolution of the state of the ConstrainedIntegrator system.
    :param x:   current state
    :param u:   control input
    :return:    flow at x under u
    """

    position_x_dot = x[3]*np.cos(x[2]) - x[4]*np.sin(x[2])
    position_y_dot = x[3]*np.sin(x[2]) + x[4]*np.cos(x[2])
    orientation_dot = x[5]
    velocity_x_dot = u[0]
    velocity_y_dot = u[1]
    velocity_orientation_dot = u[2]

    return cas.vertcat(position_x_dot, position_y_dot, orientation_dot,
                       velocity_x_dot, velocity_y_dot, velocity_orientation_dot)


def stage_cost(x: Union[np.array, cas.SX.sym], u: Union[np.array, cas.SX.sym]):
    """
    Stage cost function penalizing the control effort.
    :param x:   state (dummy input required for consistency)
    :param u:   control input
    :return:    stage cost incurred at (x, u)
    """
    cost = u[0]**2 + u[1]**2 + u[2]**2
    return cost


def terminal_cost(x: Union[np.array, cas.SX.sym], xf: np.array):
    """
    Terminal cost function penalizing the deviation from a desired final state xf.
    :param x:   (final) state
    :param xf:  desired final state
    :return:
    """
    cost = 10*(x[0]-xf[0])**2 + (x[1]-xf[1])**2 + (x[3]-xf[3])**2
    return cost


def ineq_con_stage(x: Union[np.array, cas.SX.sym], u: Union[np.array, cas.SX.sym]) -> List:
    """
    Inequality constraint functions modeling the constraints from F. Altché et al., 'A Simple Dynamic Model for
    Aggressive , Near-Limits Trajectory Planning' in the standard form h(x,u) <= 0 (this function only returns the
    values of  h(x,u)).
    :param x:   state
    :param u:   control input
    :return:    values of h(x,u) at the given state and control input
    """
    A = np.array([[2.6, 1, 0], [2.6, -1, 0]])
    b = np.array([[15.3], [15.3]])
    alpha = 9.4
    beta = 9.0
    gamma = 0.56
    ineq_con = [(u[0] / alpha) ** 2 + (u[1] / beta) ** 2 - 1,
                np.matmul(A, u) - b,
                gamma * u[1] - u[2],
                -gamma * u[1] + u[2]]
    return ineq_con


def ineq_con_terminal(x: Union[np.array, cas.SX.sym]) -> List:
    """
    (Terminal) Inequality constraint functions modeling the terminal constraints in the standard form h(x) <= 0 (this
    function only returns the values of  h(x)).
    :param x:   (terminal) state
    :return:    values of h(x) at the given state
    """

    ineq_con = [0 - x[3]]
    return ineq_con
