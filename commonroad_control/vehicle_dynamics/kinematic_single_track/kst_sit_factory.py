from typing import Union, Any

import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState, KSTStateIndices
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput, KSTInputIndices


class KSTSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Kinematic single track model factory for state, input, and trajectory.
    """
    def state_from_numpy_array(
            self,
            x_np: np.array,
    ) -> Union['KSTState']:
        """
        Set values of class from a given array.
        :param x_np: state vector - array of dimension (dim,)
        """
        if int(x_np.shape[0]) != 6:
            raise ValueError(f'Dimension {x_np.shape[0]} does not match required {KSTStateIndices.dim}')
        if x_np.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,1) but is {x_np.ndim}")

        return KSTState(
            position_x=x_np[KSTStateIndices.position_x],
            position_y=x_np[KSTStateIndices.position_y],
            velocity=x_np[KSTStateIndices.velocity],
            heading=x_np[KSTStateIndices.heading],
            steering_angle=x_np[KSTStateIndices.steering_angle],
        )

    def input_from_numpy_array(
            self,
            u_np: np.array
    ) -> Union['KSTInput']:
        """
        Set values from a given array.
        :param u_np: control input - array of dimension (self.dim,)
        """
        if u_np.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,) but is {u_np.ndim}")
        if u_np.shape[0] != KSTInputIndices.dim:
            raise ValueError(f"input should be ({KSTStateIndices.dim},) but is {u_np.shape[0]}")

        return KSTInput(
            acceleration=u_np[KSTInputIndices.acceleration],
            steering_angle_velocity=u_np[KSTInputIndices.steering_angle_velocity]
        )




