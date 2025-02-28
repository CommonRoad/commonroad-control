from typing import Union, Any

import numpy as np

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput, DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBStateIndices, DBState
from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface



class DSTSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Dynamic Single Factory for State, Input and Trajectory
    """
    def state_from_numpy_array(
            self,
            arr: np.ndarray,
    ) -> Union['DBState']:
        """
        Set values of class from a given array.
        :param arr: state vector - array of dimension (dim,)
        :return: dst state
        """
        if arr.shape[0] != DBStateIndices.dim:
            raise ValueError(f'Dimension {arr.shape[0]} does not match')
        if arr.size > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {arr}")

        return DBState(
            position_x=arr[DBStateIndices.position_x],
            position_y=arr[DBStateIndices.position_y],
            velocity_long=arr[DBStateIndices.velocity_long],
            velocity_lat=arr[DBStateIndices.velocity_lat],
            acceleration=arr[DBStateIndices.acceleration],
            heading=arr[DBStateIndices.heading],
            yaw_rate=arr[DBStateIndices.yaw_rate],
            steering_angle=arr[DBStateIndices.steering_angle]
        )


    def input_from_numpy_array(
            self,
            arr: np.ndarray
    ) -> Union['DBInput']:
        """
        Set values from a given array.
        :param arr: input vector - array of dimension (self.dim,)
        :return: DST input
        """
        if arr.size > 1:
            raise ValueError(f"size of np_array should be (dim,) but is {arr}")
        if arr.shape[0] != DBStateIndices.dim:
            raise ValueError(f"input should be ({DBInputIndices.dim},) but is {arr}")

        return DBInput(
            jerk=arr[DBInputIndices.jerk],
            steering_angle_velocity=arr[DBInputIndices.steering_angle_velocity]
        )




