import enum
from typing import Callable, Union

import casadi as cas
import numpy as np


@enum.unique
class TrajectoryMode(enum.Enum):
    """
    Types of points for vehicle model trajectories.
    """

    State = "state"
    Input = "input"
    Disturbance = "disturbance"


def rk4_integrator(
    x0: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    w: Union[np.ndarray, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ode: Callable,
    t_final: float,
) -> Union[np.ndarray, cas.SX.sym]:
    """
    This function implements the classic Runge-Kutta method (aka RK4), an explicit fourth-order method for numerical
    integration.
    :param x0: initial state for integration (at time t=0) - CasADi symbolic/ array
    :param u: control input - CasADi symbolic/ array
    :param w: disturbance - CasADi symbolic/ array
    :param ode: ordinary differential equation describing the flow of a dynamical system
    :param t_final: time horizon for integration - float
    :return: system state at time t=dt
    """

    if t_final < 0:
        raise ValueError("Time horizon for integration must be non-negative.")

    k1 = ode(x0, u, w)
    k2 = ode(x0 + t_final / 2 * k1, u, w)
    k3 = ode(x0 + t_final / 2 * k2, u, w)
    k4 = ode(x0 + t_final * k3, u, w)

    return x0 + t_final / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def gt_tol(a: float, b: float, rel_tol: float = 1e-12, abs_tol: float = 1e-15):
    """
    Checks whether a > b considering floating-point arithmetic using a combination of absolute (for numbers close to zero) and relative tolerance (rel_tol * max(|a|, |b|)).
    :param a: first number for comparison - float
    :param b: second number for comparison - float
    :param rel_tol: relative tolerance  - float
    :param abs_tol: absolute tolerance - float
    :return: True if `a` is greater than `b` beyond the computed tolerance, False otherwise.
    """
    if rel_tol < 0:
        raise ValueError("Relative tolerance must be strictly positive")
    if abs_tol <= 0:
        raise ValueError("Absolute tolerance must be strictly positive")

    tol = max(abs_tol, rel_tol * max(abs(a), abs(b)))
    return a > b + tol


def lt_tol(a, b, rel_tol=1e-12, abs_tol=1e-15):
    """
    Checks whether a < b considering floating-point arithmetic using a combination of absolute (for numbers close to zero) and relative tolerance (rel_tol * max(|a|, |b|)).
    :param a: first number for comparison - float
    :param b: second number for comparison - float
    :param rel_tol: relative tolerance  - float
    :param abs_tol: absolute tolerance - float
    :return: True if `a` is less than `b` beyond the computed tolerance, False otherwise.
    """
    if rel_tol < 0:
        raise ValueError("Relative tolerance must be strictly positive")
    if abs_tol <= 0:
        raise ValueError("Absolute tolerance must be strictly positive")

    tol = max(abs_tol, rel_tol * max(abs(a), abs(b)))
    return a < b - tol
