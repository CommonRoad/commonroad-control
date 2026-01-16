import logging
from typing import Dict, List, Union

import numpy as np

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_disturbance import (
    DBDisturbance,
    DBDisturbanceIndices,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import (
    DBInput,
    DBInputIndices,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import (
    DBState,
    DBStateIndices,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import (
    DBTrajectory,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

logger = logging.getLogger(__name__)


class DBSIDTFactory(StateInputDisturbanceTrajectoryFactoryInterface):
    """
    Factory for creating dynamic bicycle model State, Input, Disturbance, and Trajectory.
    """

    state_dimension: int = DBStateIndices.dim
    input_dimension: int = DBInputIndices.dim
    disturbance_dimension: int = DBDisturbanceIndices.dim

    @classmethod
    def state_from_args(
        cls,
        position_x: float,
        position_y: float,
        velocity_long: float,
        velocity_lat: float,
        heading: float,
        yaw_rate: float,
        steering_angle: float,
    ) -> Union["DBState"]:
        """
        Create State from args
        :param position_x: position x of center of gravity (Cartesian coordinates)
        :param position_y: position y of center of gravity (Cartesian coordinates)
        :param velocity_long: longitudinal velocity (body frame)
        :param velocity_lat: lateral velocity (body frame)
        :param heading: heading
        :param yaw_rate: yaw rate
        :param steering_angle: steering angle
        :return: DBState
        """
        return DBState(
            position_x=position_x,
            position_y=position_y,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
            heading=heading,
            yaw_rate=yaw_rate,
            steering_angle=steering_angle,
        )

    @classmethod
    def input_from_args(
        cls, acceleration: float, steering_angle_velocity: float
    ) -> Union["DBInput"]:
        """
        Create Input from args
        :param acceleration: longitudinal acceleration
        :param steering_angle_velocity: lateral acceleration
        :return: DBInput
        """
        return DBInput(
            acceleration=acceleration, steering_angle_velocity=steering_angle_velocity
        )

    @classmethod
    def disturbance_from_args(
        cls,
        position_x: float = 0.0,
        position_y: float = 0.0,
        velocity_long: float = 0.0,
        velocity_lat: float = 0.0,
        heading: float = 0.0,
        yaw_rate: float = 0.0,
        steering_angle: float = 0.0,
    ) -> Union["DBDisturbance"]:
        """
        Create Disturbance from args - the default value of all variables is zero.
        :param position_x: position x of center of gravity (Cartesian coordinates)
        :param position_y: position y of center of gravity (Cartesian coordinates)
        :param velocity_long: longitudinal velocity (body frame)
        :param velocity_lat: lateral velocity (body frame)
        :param heading: heading
        :param yaw_rate: yaw rate
        :param steering_angle: steering angle
        :return: DBDisturbance
        """
        return DBDisturbance(
            position_x=position_x,
            position_y=position_y,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
            heading=heading,
            yaw_rate=yaw_rate,
            steering_angle=steering_angle,
        )

    @classmethod
    def state_from_numpy_array(
        cls,
        x_np: np.ndarray[tuple[float], np.dtype[np.float64]],
    ) -> Union["DBState"]:
        """
        Create State from numpy array
        :param x_np: state vector - array of dimension (cls.state_dimension,)
        :return: DBState
        """

        if x_np.ndim > 1 or x_np.shape[0] != cls.state_dimension:
            logger.error(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )

        return DBState(
            position_x=x_np[DBStateIndices.position_x],
            position_y=x_np[DBStateIndices.position_y],
            velocity_long=x_np[DBStateIndices.velocity_long],
            velocity_lat=x_np[DBStateIndices.velocity_lat],
            heading=x_np[DBStateIndices.heading],
            yaw_rate=x_np[DBStateIndices.yaw_rate],
            steering_angle=x_np[DBStateIndices.steering_angle],
        )

    @classmethod
    def input_from_numpy_array(
        cls, u_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DBInput"]:
        """
        Create Input from numpy array
        :param u_np: control input - array of dimension (cls.input_dimension,)
        :return: DBInput
        """

        if u_np.ndim > 1 or u_np.shape[0] != cls.input_dimension:
            logger.error(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )

        return DBInput(
            acceleration=u_np[DBInputIndices.acceleration],
            steering_angle_velocity=u_np[DBInputIndices.steering_angle_velocity],
        )

    @classmethod
    def disturbance_from_numpy_array(
        cls, w_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DBDisturbance"]:
        """
        Create Disturbance from numpy array
        :param w_np: disturbance - array of dimension (cls.disturbance_dimension,)
        :return: DBDisturbance
        """

        if w_np.ndim > 1 or w_np.shape[0] != cls.disturbance_dimension:
            logger.error(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )

        return DBDisturbance(
            position_x=w_np[DBDisturbanceIndices.position_x],
            position_y=w_np[DBDisturbanceIndices.position_y],
            velocity_long=w_np[DBDisturbanceIndices.velocity_long],
            velocity_lat=w_np[DBDisturbanceIndices.velocity_lat],
            heading=w_np[DBDisturbanceIndices.heading],
            yaw_rate=w_np[DBDisturbanceIndices.yaw_rate],
            steering_angle=w_np[DBDisturbanceIndices.steering_angle],
        )

    @classmethod
    def trajectory_from_points(
        cls,
        trajectory_dict: Union[Dict[int, DBState], Dict[int, DBInput]],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> DBTrajectory:
        """
        Create State, Input, or Disturbance Trajectory from list of DB points.
        :param trajectory_dict: dict of time steps to kb points
        :param mode: type of points (State, Input, or Disturbance)
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: DBTrajectory
        """
        return DBTrajectory(points=trajectory_dict, mode=mode, t_0=t_0, delta_t=delta_t)

    @classmethod
    def trajectory_from_numpy_array(
        cls,
        traj_np: np.ndarray[tuple[float, float], np.dtype[np.float64]],
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> DBTrajectory:
        """
        Create State, Input, or Disturbance Trajectory from numpy array.
        :param traj_np: numpy array storing the values of the point variables
        :param mode: type of points (State, Input, or Disturbance)
        :param time_steps: time steps of the points in the columns of traj_np
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: DBTrajectory
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

        return DBTrajectory(
            points=dict(zip(time_steps, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0,
        )
