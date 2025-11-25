from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Union, List, Dict

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode


class StateInputDisturbanceTrajectoryFactoryInterface(ABC):
    state_dimension: int
    input_dimension: int
    disturbance_dimension: int

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

    @staticmethod
    @abstractmethod
    def disturbance_from_args(
            *args
    ) -> Union[Any]:
        pass

    @abstractmethod
    def state_from_numpy_array(
            self,
            x_np: np.ndarray,
    ) -> Union[Any]:
        pass

    @abstractmethod
    def input_from_numpy_array(
            self,
            u_np: np.ndarray
    ) -> Union[Any]:
        pass

    @classmethod
    @abstractmethod
    def disturbance_from_numpy_array(
            cls,
            w_np: np.ndarray
    ) -> Union[Any]:
        pass

    @abstractmethod
    def trajectory_from_state_or_input(self,
                                       trajectory_dict: Union[Dict[int, StateInterface], Dict[int, InputInterface]],
                                       mode: TrajectoryMode,
                                       t_0: float,
                                       delta_t: float
                                       ) -> TrajectoryInterface:
        pass

    @abstractmethod
    def trajectory_from_numpy_array(
            self,
            traj_np: np.ndarray,
            mode: TrajectoryMode,
            time_steps: List[int],
            t_0: float,
            delta_t: float
    ) -> TrajectoryInterface:
        pass