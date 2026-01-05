from __future__ import annotations

import logging
from typing import Dict, List, Union

import numpy as np

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_disturbance import (
    KBDisturbance,
    KBDisturbanceIndices,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import (
    KBInput,
    KBInputIndices,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import (
    KBState,
    KBStateIndices,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import (
    KBTrajectory,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

logger = logging.getLogger(__name__)


class KBSIDTFactory(StateInputDisturbanceTrajectoryFactoryInterface):
    """
    Factory for creating kinematic bicycle model State, Input, Disturbance, and Trajectory.
    """

    state_dimension: int = KBStateIndices.dim
    input_dimension: int = KBInputIndices.dim
    disturbance_dimension: int = KBDisturbanceIndices.dim

    @classmethod
    def state_from_args(
        cls,
        position_x: float,
        position_y: float,
        velocity: float,
        heading: float,
        steering_angle: float,
    ) -> Union["KBState"]:
        """
        Create State from args
        :param position_x: position x of center of gravity (Cartesian coordinates)
        :param position_y: position y of center of gravity (Cartesian coordinates)
        :param velocity: velocity
        :param heading: heading
        :param steering_angle: steering angle
        :return: KBState
        """
        return KBState(
            position_x=position_x,
            position_y=position_y,
            velocity=velocity,
            heading=heading,
            steering_angle=steering_angle,
        )

    @classmethod
    def input_from_args(
        cls,
        acceleration: float,
        steering_angle_velocity,
    ) -> Union["KBInput"]:
        """
        Create Input from args
        :param acceleration: longitudinal acceleration
        :param steering_angle_velocity: steering angle velocity
        :return: KBInput
        """
        return KBInput(acceleration=acceleration, steering_angle_velocity=steering_angle_velocity)

    @staticmethod
    def disturbance_from_args(
        position_x: float = 0.0,
        position_y: float = 0.0,
        velocity: float = 0.0,
        heading: float = 0.0,
        steering_angle: float = 0.0,
    ) -> Union["KBDisturbance"]:
        """
        Create Disturbance from args - the default value of all variables is zero.
        :param position_x: position x of center of gravity
        :param position_y: position y of center of gravity
        :param velocity: velocity
        :param heading: heading
        :param steering_angle: steering angle
        :return: KBDisturbance
        """
        return KBDisturbance(
            position_x=position_x,
            position_y=position_y,
            velocity=velocity,
            heading=heading,
            steering_angle=steering_angle,
        )

    @classmethod
    def state_from_numpy_array(
        cls,
        x_np: np.ndarray[tuple[float], np.dtype[np.float64]],
    ) -> Union["KBState"]:
        """
        Create State from numpy array
        :param x_np: state vector - array of dimension (cls.state_dimension,)
        :return: KBState
        """

        if x_np.ndim > 1 or x_np.shape[0] != cls.state_dimension:
            logger.error(f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}")
            raise ValueError(f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}")

        return KBState(
            position_x=x_np[KBStateIndices.position_x],
            position_y=x_np[KBStateIndices.position_y],
            velocity=x_np[KBStateIndices.velocity],
            heading=x_np[KBStateIndices.heading],
            steering_angle=x_np[KBStateIndices.steering_angle],
        )

    @classmethod
    def input_from_numpy_array(cls, u_np: np.ndarray[tuple[float], np.dtype[np.float64]]) -> Union["KBInput"]:
        """
        Create Input from numpy array
        :param u_np: control input - array of dimension (cls.input_dimension,)
        :return: KBInput
        """

        if u_np.ndim > 1 or u_np.shape[0] != cls.input_dimension:
            logger.error(f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}")
            raise ValueError(f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}")

        return KBInput(
            acceleration=u_np[KBInputIndices.acceleration],
            steering_angle_velocity=u_np[KBInputIndices.steering_angle_velocity],
        )

    @classmethod
    def disturbance_from_numpy_array(
        cls, w_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["KBDisturbance"]:
        """
        Create Disturbance from numpy array
        :param w_np: disturbance - array of dimension (cls.disturbance_dimension,)
        :return: KBDisturbance
        """

        if w_np.ndim > 1 or w_np.shape[0] != cls.disturbance_dimension:
            logger.error(f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}")
            raise ValueError(f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}")

        return KBDisturbance(
            position_x=w_np[KBDisturbanceIndices.position_x],
            position_y=w_np[KBDisturbanceIndices.position_y],
            velocity=w_np[KBDisturbanceIndices.velocity],
            heading=w_np[KBDisturbanceIndices.heading],
            steering_angle=w_np[KBDisturbanceIndices.steering_angle],
        )

    @classmethod
    def trajectory_from_points(
        cls,
        trajectory_dict: Union[Dict[int, KBState], Dict[int, KBInput]],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> "KBTrajectory":
        """
        Create State, Input, or Disturbance Trajectory from list of KB points.
        :param trajectory_dict: dict of time steps to kb points
        :param mode: type of points (State, Input, or Disturbance)
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: KBTrajectory
        """
        return KBTrajectory(points=trajectory_dict, mode=mode, t_0=t_0, delta_t=delta_t)

    @classmethod
    def trajectory_from_numpy_array(
        cls,
        traj_np: np.ndarray[tuple[float, float], np.dtype[np.float64]],
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> "KBTrajectory":
        """
        Create State, Input, or Disturbance Trajectory from numpy array.
        :param traj_np: numpy array storing the values of the point variables
        :param mode: type of points (State, Input, or Disturbance)
        :param time_steps: time steps of the points in the columns of traj_np
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: KBTrajectory
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

        return KBTrajectory(
            points=dict(zip(time_steps, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0,
        )
