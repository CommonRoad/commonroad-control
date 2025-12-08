import logging
from typing import Dict, List, Union

import numpy as np

from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import (
    DIDisturbance,
    DIDisturbanceIndices,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_input import (
    DIInput,
    DIInputIndices,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_state import (
    DIState,
    DIStateIndices,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_trajectory import (
    DITrajectory,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

logger = logging.getLogger(__name__)


class DISIDTFactory(StateInputDisturbanceTrajectoryFactoryInterface):
    """
    Factory for creating double integrator model State, Input, Disturbance, and Trajectory.
    """

    state_dimension = DIStateIndices.dim
    input_dimension = DIInputIndices.dim
    disturbance_dimension = DIDisturbanceIndices.dim

    @classmethod
    def state_from_args(
        cls,
        position_long: float,
        position_lat: float,
        velocity_long: float,
        velocity_lat: float,
    ) -> Union["DIState"]:
        """
        Create State from args
        :param position_long: longitudinal position
        :param position_lat: lateral position
        :param velocity_long: longitudinal velocity
        :param velocity_lat: lateral velocity
        :return: DIState
        """
        return DIState(
            position_long=position_long,
            position_lat=position_lat,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
        )

    @classmethod
    def input_from_args(
        cls, acceleration_long: float, acceleration_lat: float
    ) -> Union["DIInput"]:
        """
        Create Input from args
        :param acceleration_long: longitudinal acceleration
        :param acceleration_lat: lateral acceleration
        :return: DIInput
        """
        return DIInput(
            acceleration_long=acceleration_long, acceleration_lat=acceleration_lat
        )

    @classmethod
    def disturbance_from_args(
        cls,
        position_long: float = 0.0,
        position_lat: float = 0.0,
        velocity_long: float = 0.0,
        velocity_lat: float = 0.0,
    ) -> Union["DIDisturbance"]:
        """
        Create Disturbance from args - the default value of all variables is zero.
        :param position_long: longitudinal position
        :param position_lat: lateral position
        :param velocity_long: longitudinal velocity
        :param velocity_lat: lateral velocity
        :return: DIDisturbance
        """
        return DIDisturbance(
            position_long=position_long,
            position_lat=position_lat,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
        )

    @classmethod
    def state_from_numpy_array(
        cls, x_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DIState"]:
        """
        Create State from numpy array
        :param x_np: state vector - array of dimension (cls.state_dimension,)
        :return: DIState
        """

        if x_np.ndim > 1 or x_np.shape[0] != cls.state_dimension:
            logger.error(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )

        return DIState(
            position_long=x_np[DIStateIndices.position_long],
            position_lat=x_np[DIStateIndices.position_lat],
            velocity_long=x_np[DIStateIndices.velocity_long],
            velocity_lat=x_np[DIStateIndices.velocity_lat],
        )

    @classmethod
    def input_from_numpy_array(
        cls, u_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DIInput"]:
        """
        Create Input from numpy array
        :param u_np: control input - array of dimension (cls.input_dimension,)
        :return: DIInput
        """

        if u_np.ndim > 1 or u_np.shape[0] != cls.input_dimension:
            logger.error(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )

        return DIInput(
            acceleration_long=u_np[DIInputIndices.acceleration_long],
            acceleration_lat=u_np[DIInputIndices.acceleration_lat],
        )

    @classmethod
    def disturbance_from_numpy_array(
        cls, w_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DIDisturbance"]:
        """
        Create Disturbance from numpy array
        :param w_np: disturbance - array of dimension (cls.disturbance_dimension,)
        :return: DIDisturbance
        """

        if w_np.ndim > 1 or w_np.shape[0] != cls.disturbance_dimension:
            logger.error(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )

        return DIDisturbance(
            position_long=w_np[DIDisturbanceIndices.position_long],
            position_lat=w_np[DIDisturbanceIndices.position_lat],
            velocity_long=w_np[DIDisturbanceIndices.velocity_long],
            velocity_lat=w_np[DIDisturbanceIndices.velocity_lat],
        )

    @classmethod
    def trajectory_from_points(
        cls,
        trajectory_dict: Union[Dict[int, DIState], Dict[int, DIInput]],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> DITrajectory:
        """
        Create State, Input, or Disturbance Trajectory from list of DI points.
        :param trajectory_dict: dict of time steps to kb points
        :param mode: type of points (State, Input, or Disturbance)
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: DITrajectory
        """
        return DITrajectory(points=trajectory_dict, mode=mode, t_0=t_0, delta_t=delta_t)

    @classmethod
    def trajectory_from_numpy_array(
        cls,
        traj_np: np.ndarray[tuple[float, float], np.dtype[np.float64]],
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> DITrajectory:
        """
        Create State, Input, or Disturbance Trajectory from numpy array.
        :param traj_np: numpy array storing the values of the point variables
        :param mode: type of points (State, Input, or Disturbance)
        :param time_steps: time steps of the points in the columns of traj_np
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: DITrajectory
        """

        # convert trajectory points to State/Input/DisturbanceInterface
        points_val = []
        for kk in range(len(time_steps)):
            if mode == TrajectoryMode.State:
                points_val.append(cls.state_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Input:
                points_val.append(cls.input_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Disturbance:
                points_val.append(cls.disturbance_from_numpy_array(traj_np[:, kk]))

        return DITrajectory(
            points=dict(zip(time_steps, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0,
        )
