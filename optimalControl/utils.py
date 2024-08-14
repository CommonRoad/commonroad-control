import numpy as np
import sys
from typing import Union, Callable, Dict, Type
import casadi as cas
from optimalControl.ocp_dataclasses import State, ControlInput, TrajectoryPoint


def rk4_integrator(x0: Union[np.array, cas.SX.sym], u: Union[np.array, cas.SX.sym], ode: Callable, dt: float) \
        -> Union[np.array, cas.SX.sym]:
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
    k2 = ode(x0 + dt/2*k1, u)
    k3 = ode(x0 + dt/2*k2, u)
    k4 = ode(x0 + dt*k3, u)
    return x0 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)


class Trajectory:
    def __init__(self, N: int, point_type: ['State', 'ControlInput'], system: str):
        """
        :param N:           (discrete) prediction horizon aka length of the trajectory
        :param point_type:  string indicating the type of trajectory points (State/ControlInput/...)
        :param system:      name of the dynamical system (required for setting the correct trajectory points)
        """
        if point_type == 'State':
            self._trajectory = dict.fromkeys(range(N + 1))
            self._point_type = 'State' + system
        elif point_type == 'ControlInput':
            self._trajectory = dict.fromkeys(range(N))
            self._point_type = 'ControlInput' + system

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
        self._trajectory[time_step] = getattr(sys.modules[__name__], self._point_type)()
        self._trajectory[time_step].set_values_from_array(point_np)

    def set_trajectory_from_array(self, trajectory_np: np.array):
        """
        Converts a numpy array storing trajectory points as columns (e.g., the solution of an optimal control problem)
        into a Trajectory object.
        :param trajectory_np:   trajectory represented as a numpy array,
        """
        for kk in range(len(self._trajectory)):
            self.set_point_from_array_at_time_step(trajectory_np[:, kk], kk)


class TrajectoryNumpyConverter:
    def __init__(self, dc2npidx_dict: Dict, dc_type: str):
        """
        :param dc2npidx_dict:   dict providing the indices of the fields of State/Control Input objects in the
                                corresponding numpy array
        :param dc_type:         string indicating the dataclass for conversion from/to a given numpy array
        """
        self._dim = len(dc2npidx_dict)
        self._dc2npidx_dict = dc2npidx_dict
        if dc_type in ['State', 'ControlInput']:
            self._dc_type = dc_type
        else:
            raise ValueError('Conversion not defined for dataclass ' + dc_type + '!')

    def convert_to_numpy(self, y_dc: Union[State, ControlInput]) -> np.array:
        """
        Converts a given trajectory point (represented as a dataclass object) into a numpy array for numerical
        optimization.
        :param y_dc: instance of dataclass State/ControlInput
        :return: numpy array representation of y_dc (dimension (y_dc.dim,1))
        """
        y_np = np.zeros((self._dim, 1), dtype=float)
        # set field values of dataclass instance y_dc as corresponding entries of array y_np
        for field in self._dc2npidx_dict.keys():
            y_np[self._dc2npidx_dict[field]] = getattr(y_dc, field)

        return y_np

    def convert_from_numpy(self, y_np: np.array) -> Union[State, ControlInput]:
        """
        Converts a numpy array representing a trajectory point (State/ControlInput) into an instance of the
        corresponding dataclass.
        :param y_np: numpy array containing the values of the states/control
        :return: instance of dataclass State/ControlInput representing y_np
        """
        y_dc = getattr(sys.modules[__name__], self._dc_type)(dim=self._dim)
        # copy entries of array y_np to corresponding fields of dataclass instance y_dc
        for field in self._dc2npidx_dict.keys():
            setattr(y_dc, field, y_np[self._dc2npidx_dict[field], 0])

        return y_dc
