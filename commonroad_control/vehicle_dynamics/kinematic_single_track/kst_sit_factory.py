from typing import Union, Dict, List

import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState, KSTStateIndices
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput, KSTInputIndices


class KSTSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Kinematic single track model factory for state, input, and trajectory.
    """
    def state_from_args(
            self,
            position_x: float,
            position_y: float,
            velocity: float,
            heading: float,
            steering_angle: float,
    ) -> Union['KSTState']:
        """
        Create KST state from args
        :param position_x: position x
        :param position_y: position y
        :param velocity: velocity
        :param heading: heading from vehicle center
        :param steering_angle: steering angle
        :return: KSTState
        """
        return KSTState(
            position_x=position_x,
            position_y=position_y,
            velocity=velocity,
            heading=heading,
            steering_angle=steering_angle
        )

    def input_from_args(
            self,
            acceleration: float,
            steering_angle_velocity,
    ) -> Union['KSTInput']:
        """
        Return KST input
        :param acceleration: input acceleration
        :param steering_angle_velocity: input steering angle velocity
        :return: KSTInput
        """
        return KSTInput(
            acceleration=acceleration,
            steering_angle_velocity=steering_angle_velocity
        )


    def state_from_numpy_array(
            self,
            x_np: np.array,
    ) -> Union['KSTState']:
        """
        Set values of class from a given array.
        :param x_np: state vector - array of dimension (dim,)
        """
        if int(x_np.shape[0]) != KSTStateIndices.dim:
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


    def trajectory_from_state_or_input(
            self,
            trajectory_dict: Union[Dict[int, KSTState], Dict[int, KSTInput]],
            mode: TrajectoryMode,
            t_0: float,
            delta_t: float
    ) -> KSTTrajectory:
        """
        Build trajectory from kst state or input
        :param trajectory_dict: dict of time steps to kst points
        :param mode:
        :param t_0:
        :param delta_t:
        :return: KST-Trajectory
        """
        return KSTTrajectory(
                points=trajectory_dict,
                mode=mode,
                t_0=t_0,
                delta_t=delta_t
            )

    def trajectory_from_numpy_array(
            self,
            traj_np: np.array,
            mode: TrajectoryMode,
            time: List[int],
            t_0: float,
            delta_t: float
    ) -> KSTTrajectory:
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

        return KSTTrajectory(
            points=dict(zip(time, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0
        )








