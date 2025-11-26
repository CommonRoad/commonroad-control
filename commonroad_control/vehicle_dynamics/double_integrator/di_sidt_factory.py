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
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import (
    DBTrajectory,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

logger = logging.getLogger(__name__)


class DISIDTFactoryDisturbance(StateInputDisturbanceTrajectoryFactoryInterface):
    """
    Double integrator model factory for state, input, and trajectory.
    """

    state_dimension = DIStateIndices.dim
    input_dimension = DIInputIndices.dim
    disturbance_dimension = DIDisturbanceIndices.dim

    def state_from_args(
        self,
        position_long: float,
        position_lat: float,
        velocity_long: float,
        velocity_lat: float,
    ) -> Union["DIState"]:
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
            velocity_lat=velocity_lat,
        )

    def input_from_args(
        self, acceleration_long: float, acceleration_lat: float
    ) -> Union["DIInput"]:
        """
        Create DI input from args.
        :param acceleration_long: longitudinal acceleration
        :param acceleration_lat: lateral acceleration
        :return: DIInput
        """
        return DIInput(
            acceleration_long=acceleration_long, acceleration_lat=acceleration_lat
        )

    @staticmethod
    def disturbance_from_args(
        position_long: float = 0.0,
        position_lat: float = 0.0,
        velocity_long: float = 0.0,
        velocity_lat: float = 0.0,
    ) -> Union["DIDisturbance"]:
        """
        Create DB disturbance from args.
        :param position_long:
        :param position_lat:
        :param velocity_long:
        :param velocity_lat:
        :return: DIDisturbance
        """
        return DIDisturbance(
            position_long=position_long,
            position_lat=position_lat,
            velocity_long=velocity_long,
            velocity_lat=velocity_lat,
        )

    def state_from_numpy_array(
        self, x_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DIState"]:
        """
        Set values of state from a given array
        :param x_np: state - array of dimension
        :return: state object
        """

        if int(x_np.shape[0]) != DIStateIndices.dim:
            logger.error(
                f"Dimension {x_np.shape[0]} does not match required {DIStateIndices.dim}"
            )
            raise ValueError(
                f"Dimension {x_np.shape[0]} does not match required {DIStateIndices.dim}"
            )
        if x_np.ndim > 1:
            logger.error(f"ndim of np_array should be (dim,1) but is {x_np.ndim}")
            raise ValueError(f"ndim of np_array should be (dim,1) but is {x_np.ndim}")

        return DIState(
            position_long=x_np[DIStateIndices.position_long],
            position_lat=x_np[DIStateIndices.position_lat],
            velocity_long=x_np[DIStateIndices.velocity_long],
            velocity_lat=x_np[DIStateIndices.velocity_lat],
        )

    def input_from_numpy_array(
        self, u_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["DIInput"]:
        """
        Set values of control input from a given array.
        :param u_np: input vector - array of dimension (self.dim,)
        """
        if u_np.ndim > 1:
            logger.error(f"ndim of np_array should be (dim,) but is {u_np.ndim}")
            raise ValueError(f"ndim of np_array should be (dim,) but is {u_np.ndim}")
        if u_np.shape[0] != DIInputIndices.dim:
            logger.error(
                f"input should be ({DIInputIndices.dim},) but is {u_np.shape[0]}"
            )
            raise ValueError(
                f"input should be ({DIInputIndices.dim},) but is {u_np.shape[0]}"
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
        Sets values of double integrator disturbance from a given array.
        :param w_np: disturbance - array of dimension (DIDisturbanceIndices.dim,)
        :return: di disturbance
        """

        if w_np.shape[0] != cls.disturbance_dimension:
            logger.error(f"Dimension {w_np.shape[0]} does not match")
            raise ValueError(f"Dimension {w_np.shape[0]} does not match")
        if w_np.ndim > 1:
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

    def trajectory_from_state_or_input(
        self,
        trajectory_dict: Union[Dict[int, DIState], Dict[int, DIInput]],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> DBTrajectory:
        """
        Build trajectory from di state or input
        :param trajectory_dict: dict of time steps to kst points
        :param mode:
        :param t_0:
        :param delta_t:
        :return: KST-Trajectory
        """
        return DBTrajectory(points=trajectory_dict, mode=mode, t_0=t_0, delta_t=delta_t)

    def trajectory_from_numpy_array(
        self,
        traj_np: np.ndarray[tuple[float, float], np.dtype[np.float64]],
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> DITrajectory:
        """

        :param traj_np:
        :param mode:
        :param time_steps:
        :param t_0:
        :param delta_t:
        :return:
        """
        # convert trajectory to State/InputInterface
        points_val = []
        for kk in range(len(time_steps)):
            if mode == TrajectoryMode.State:
                points_val.append(self.state_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Input:
                points_val.append(self.input_from_numpy_array(traj_np[:, kk]))

        return DITrajectory(
            points=dict(zip(time_steps, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0,
        )
