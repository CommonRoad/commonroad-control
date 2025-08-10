from typing import Union, List, Any
import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.double_integrator.di_trajectory import DITrajectory
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices


class DISITFactory(StateInputTrajectoryFactoryInterface):
    """
    Double integrator model factory for state, input, and trajectory.
    """

    def state_from_args(
            self,
            position_long: float,
            position_lat: float,
            velocity_long: float,
            velocity_lat: float
    ) -> Union['DIState']:
        """
        Create DI state from args.
        :param position_long: longitudinal position
        :param position_lat: lateral position
        :param velocity_long: longitudinal veloctiy
        :param velocity_lat: lateral velocity
        :return: DIState
        """
        return DIState(
            position_long=position_long,
            position_lat=position_lat,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat
        )

    def input_from_args(
            self,
            acceleration_long: float,
            acceleration_lat: float
    ) -> Union['DIInput']:
        """
        Create DI input from args.
        :param acceleration_long: longitudinal acceleration
        :param acceleration_lat: lateral acceleration
        :return: DIInput
        """
        return DIInput(
            acceleration_long=acceleration_long,
            acceleration_lat=acceleration_lat
        )

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

    def trajectory_from_numpy_array(
            self,
            traj_np: np.array,
            mode: TrajectoryMode,
            time: List[int],
            t_0: float,
            delta_t: float
    ) -> DITrajectory:
        """

        :param traj_np:
        :param mode:
        :param time:
        :param t_0:
        :param delta_t:
        :return:
        """
        # convert trajectory to State/InputInterface
        points_val = []
        for kk in range(len(time)):
            if mode == TrajectoryMode.State:
                points_val.append(self.state_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Input:
                points_val.append(self.input_from_numpy_array(traj_np[:, kk]))

        return DITrajectory(
            points=dict(zip(time, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0
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



    def state_from_args(
            self,
            *args
    ) -> Union[Any]:
        """
        Create State from input args
        """
        pass

    def input_from_args(
            self,
            *args
    ) -> Union[Any]:
        """
        Return input
        """
        pass