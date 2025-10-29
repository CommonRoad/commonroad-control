from typing import Union, Dict, List

import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput, DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBStateIndices, DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory


class DBSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Dynamic bicycle model factory for state, input, and trajectory.
    """

    def state_from_args(
            self,
            position_x: float,
            position_y: float,
            velocity_long: float,
            velocity_lat: float,
            heading: float,
            yaw_rate: float,
            steering_angle: float
    ) -> Union['DBState']:
        """
        Create DB state from args.
        :param position_x:
        :param position_y:
        :param velocity_long:
        :param velocity_lat:
        :param heading:
        :param yaw_rate:
        :param steering_angle:
        :return: DBState
        """
        return DBState(
            position_x=position_x,
            position_y=position_y,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
            heading=heading,
            yaw_rate=yaw_rate,
            steering_angle=steering_angle
        )

    def input_from_args(
            self,
            acceleration: float,
            steering_angle_velocity: float
    ) -> Union['DBInput']:
        """
        Create DB input from args.
        :param acceleration: longitudinal acceleration
        :param steering_angle_velocity: lateral acceleration
        :return: DIInput
        """
        return DBInput(
            acceleration=acceleration,
            steering_angle_velocity=steering_angle_velocity
        )

    def state_from_numpy_array(
            self,
            x_np: np.ndarray[tuple[float], np.dtype[np.float64]],
    ) -> Union['DBState']:
        """
        Set values of class from a given array.
        :param x_np: state - array of dimension (dim,)
        :return: dst state
        """
        if x_np.shape[0] != DBStateIndices.dim:
            raise ValueError(f'Dimension {x_np.shape[0]} does not match')
        if x_np.ndim > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {x_np.shape}")

        return DBState(
            position_x=x_np[DBStateIndices.position_x],
            position_y=x_np[DBStateIndices.position_y],
            velocity_long=x_np[DBStateIndices.velocity_long],
            velocity_lat=x_np[DBStateIndices.velocity_lat],
            heading=x_np[DBStateIndices.heading],
            yaw_rate=x_np[DBStateIndices.yaw_rate],
            steering_angle=x_np[DBStateIndices.steering_angle]
        )

    def input_from_numpy_array(
            self,
            u_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union['DBInput']:
        """
        Set values from a given array.
        :param u_np: input vector - array of dimension (self.dim,)
        :return: DST input
        """
        if u_np.size > 1:
            raise ValueError(f"size of array should be (dim,) but is {u_np}")
        if u_np.shape[0] != DBStateIndices.dim:
            raise ValueError(f"input should be ({DBInputIndices.dim},) but is {u_np}")

        return DBInput(
            acceleration=u_np[DBInputIndices.acceleration],
            steering_angle_velocity=u_np[DBInputIndices.steering_angle_velocity]
        )

    def trajectory_from_state_or_input(
            self,
            trajectory_dict: Union[Dict[int, DBState], Dict[int, DBInput]],
            mode: TrajectoryMode,
            t_0: float,
            delta_t: float
   ) -> DBTrajectory:
        """
        Build trajectory from db state or input
        :param trajectory_dict: dict of time steps to kst points
        :param mode:
        :param t_0:
        :param delta_t:
        :return: KST-Trajectory
        """
        return DBTrajectory(
                points=trajectory_dict,
                mode=mode,
                t_0=t_0,
                delta_t=delta_t
            )

    def trajectory_from_numpy_array(
            self,
            traj_np: np.ndarray[tuple[float,float], np.dtype[np.float64]],
            mode: TrajectoryMode,
            time_steps: List[int],
            t_0: float,
            delta_t: float
    ) -> TrajectoryInterface:
        """

        :param traj_np:
        :param mode:
        :param time_steps:
        :param t_0:
        :param delta_t:
        :return:
        """

        points = []
        for kk in range(len(time_steps)):
            if mode == TrajectoryMode.State:
                points.append(self.state_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Input:
                points.append(self.input_from_numpy_array(traj_np[:, kk]))


        return DBTrajectory(
            points=dict(zip(time_steps, points)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0
        )
