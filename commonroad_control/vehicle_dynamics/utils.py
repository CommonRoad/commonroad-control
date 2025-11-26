import enum
from typing import Callable, Union

import casadi as cas
import numpy as np


@enum.unique
class TrajectoryMode(enum.Enum):
    State = "state"
    Input = "input"
    Disturbance = "disturbance"


def rk4_integrator(
    x0: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    w: Union[np.ndarray, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ode: Callable,
    dt: float,
) -> Union[np.ndarray, cas.SX.sym]:
    """
    This function implements the classic Runge-Kutta method (aka RK4), an explicit fourth-order method for numerical
    integration.
    :param x0: initial state for integration (at time t=0)
    :param u: control input
    :param w: disturbance
    :param ode: ordinary differential equation describing the flow of a
    :param dt: time step for integration
    :return: system state at time t=dt
    """

    k1 = ode(x0, u, w)
    k2 = ode(x0 + dt / 2 * k1, u, w)
    k3 = ode(x0 + dt / 2 * k2, u, w)
    k4 = ode(x0 + dt * k3, u, w)
    return x0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
