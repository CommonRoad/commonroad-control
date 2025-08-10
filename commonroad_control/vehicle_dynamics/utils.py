from typing import Union, Callable
import numpy as np
import casadi as cas
import enum


@enum.unique
class TrajectoryMode(enum.Enum):
    State = 'state'
    Input = 'mode'


def rk4_integrator(
    x0: Union[np.array, cas.SX.sym],
    u: Union[np.array, cas.SX.sym],
    ode: Callable,
    dt: float,
) -> Union[np.array, cas.SX.sym]:
    """
    This function implements the classic Runge-Kutta method (aka RK4), an explicit fourth-order method for numerical
    integration.
    :param x0:  initial state for integration (at time t=0)
    :param u:   control input
    :param ode: ordinary differential equation describing the flow of a
    :param dt:  final time for integration
    :return:    system state at time t=dt
    """

    k1 = ode(x0, u)
    k2 = ode(x0 + dt / 2 * k1, u)
    k3 = ode(x0 + dt / 2 * k2, u)
    k4 = ode(x0 + dt * k3, u)
    return x0 + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
