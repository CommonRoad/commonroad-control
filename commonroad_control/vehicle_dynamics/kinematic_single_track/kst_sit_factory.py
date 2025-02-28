from typing import Union, Any

import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState, KSTStateIndices
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput, KSTInputIndices


class KSTSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Kinematic Single Factory for State, Input and Trajectory
    """
    def state_from_numpy_array(
            self,
            arr: np.ndarray,
    ) -> Union['KSTState']:
        """
        Set values of class from a given array.
        :param arr: state vector - array of dimension (dim,)
        """
        if int(arr.shape[0]) != 6:
            raise ValueError(f'Dimension {arr.shape[0]} does not match required {KSTStateIndices.dim}')
        if arr.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,1) but is {arr.ndim}")

        return KSTState(
            position_x=arr[KSTStateIndices.position_x],
            position_y=arr[KSTStateIndices.position_y],
            velocity=arr[KSTStateIndices.velocity],
            acceleration=arr[KSTStateIndices.acceleration],
            heading=arr[KSTStateIndices.heading],
            steering_angle=arr[KSTStateIndices.steering_angle],
        )

    def input_from_numpy_array(
            self,
            arr: np.ndarray
    ) -> Union['KSTInput']:
        """
        Set values from a given array.
        :param arr: input vector - array of dimension (self.dim,)
        """
        if arr.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,) but is {arr.ndim}")
        if arr.shape[0] != KSTInputIndices.dim:
            raise ValueError(f"input should be ({KSTStateIndices.dim},) but is {arr.shape[0]}")

        return KSTInput(
            jerk=arr[KSTInputIndices.jerk],
            steering_angle_velocity=arr[KSTInputIndices.steering_angle_velocity]
        )




