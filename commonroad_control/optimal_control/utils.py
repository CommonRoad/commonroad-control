import numpy as np
import sys
from typing import Union, Callable, Dict, Type
import casadi as cas
from commonroad_control.optimal_control.ocp_dataclasses import (
    State,
    ControlInput,
    TrajectoryPoint,
)
from commonroad_control.models.constrained_integrator import (
    ImplementedModels,
    init_models,
)


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


class Trajectory:
    def __init__(
        self, N: int, point_type: ["State", "ControlInput"], system: ImplementedModels
    ):
        """
        :param N:           (discrete) prediction horizon aka length of the trajectory
        :param point_type:  string indicating the type of trajectory points (State/ControlInput/...)
        :param system:      name of the dynamical system (required for setting the correct trajectory points)
        """

        self._system = system

        if point_type == "State":
            self._trajectory = dict.fromkeys(range(N + 1))
            self._point_type = "State" + "ConstrainedIntegrator"
        elif point_type == "ControlInput":
            self._trajectory = dict.fromkeys(range(N))
            self._point_type = "ControlInput" + "ConstrainedIntegrator"

    def get_point_at_time_step(self, time_step: int) -> Type[TrajectoryPoint]:
        """
        Returns the trajectory point (State/ControlInput/...) at the given time step.
        :param time_step: (discrete) point in time
        :return: trajectory point at given time step (represented as TrajectoryPoint object)
        """
        return self._trajectory[time_step]

    def get_point_as_array_at_time_step(self, time_step: int) -> np.array:
        """
        Returns the trajectory point (State/ControlInput/...) represented as a numpy array at the given time step.
        :param time_step: (discrete) point in time
        :return: trajectory point at given time step (represented as a numpy array)
        """
        return self._trajectory[time_step].convert_to_array()

    def set_point_at_time_step(self, point_dc: Type[TrajectoryPoint], time_step: int):
        """
        Sets the given trajectory point (State/ControlInput/...) as the trajectory point at the given time step.
        :param point_dc:    trajectory point at given time step (represented as TrajectoryPoint object)
        :param time_step:   (discrete) point in time
        """

        self._trajectory[time_step] = point_dc

    def set_point_from_array_at_time_step(self, point_np: np.array, time_step: int):
        """
        Converts the given trajectory point (represented as a numpy array) into an object of the corresponding class
        and sets it as the trajectory point at the given time step.
        :param point_np:    trajectory point at given time step (represented as a numpy array)
        :param time_step:   (discrete) point in time
        """
        self._trajectory[time_step] = init_models(model=self._point_type)
        self._trajectory[time_step].set_values_from_array(point_np)

    def set_trajectory_from_array(self, trajectory_np: np.array):
        """
        Converts a numpy array storing trajectory points as columns (e.g., the solution of an optimal control problem)
        into a Trajectory object.
        :param trajectory_np:   trajectory represented as a numpy array,
        """
        for kk in range(len(self._trajectory)):
            self.set_point_from_array_at_time_step(trajectory_np[:, kk], kk)
