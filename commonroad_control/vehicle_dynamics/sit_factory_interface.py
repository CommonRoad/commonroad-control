from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Union, Literal, List

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


class StateInputTrajectoryFactoryInterface(ABC):

    @abstractmethod
    def state_from_args(
            self,
            *args
    ) -> Union[Any]:
        """
        Create State from input args
        """
        pass

    @abstractmethod
    def input_from_args(
            self,
            *args
    ) -> Union[Any]:
        """
        Return input
        """
        pass


    @abstractmethod
    def state_from_numpy_array(
            self,
            x_np: np.array,
    ) -> Union[Any]:
        pass

    @abstractmethod
    def input_from_numpy_array(
            self,
            u_np: np.array
    ) -> Union[Any]:
        pass

    @abstractmethod
    def trajectory_from_numpy_array(
            self,
            traj_np: np.array,
            mode: Literal['state', 'input'],
            time: List[int],
            t_0: float,
            delta_t: float
    ) -> TrajectoryInterface:
        pass