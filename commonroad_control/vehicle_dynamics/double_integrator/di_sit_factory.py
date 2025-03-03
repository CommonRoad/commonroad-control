from typing import Union
import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices


class DISITFactory(StateInputTrajectoryFactoryInterface):
    """
    Double integrator model factory fro state, input, and trajectory.
    """
    def state_from_numpy_array(
            self,
            x_np: np.array,
    ) -> Union['DIState']:
        """
        Set values of state from a given array
        :param x_np: state - array of dimension
        :return: state object
        """

        if int(x_np.shape[0]) != DIStateIndices.dim:
            raise ValueError(f'Dimension {x_np.shape[0]} does not match required {DIStateIndices.dim}')
        if x_np.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,1) but is {x_np.ndim}")

        return DIState(
            position_long=x_np[DIStateIndices.position_long],
            position_lat=x_np[DIStateIndices.position_lat],
            velocity_long=x_np[DIStateIndices.velocity_long],
            velocity_lat=x_np[DIStateIndices.velocity_lat]
        )

    def input_from_numpy_array(
            self,
            u_np: np.array
    ) -> Union['DIInput']:
        """
        Set values of control input from a given array.
        :param u_np: input vector - array of dimension (self.dim,)
        """
        if u_np.ndim > 1:
            raise ValueError(f"ndim of np_array should be (dim,) but is {u_np.ndim}")
        if u_np.shape[0] != DIInputIndices.dim:
            raise ValueError(f"input should be ({DIInputIndices.dim},) but is {u_np.shape[0]}")

        return DIInput(
            acceleration_long=u_np[DIInputIndices.acceleration_long],
            acceleration_lat=u_np[DIInputIndices.acceleration_lat]
        )