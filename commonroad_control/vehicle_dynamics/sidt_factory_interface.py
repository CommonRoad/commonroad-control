from abc import ABC, abstractmethod
from typing import Any, Dict, List, Union

import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode


class StateInputDisturbanceTrajectoryFactoryInterface(ABC):
    """
    Factory for creating State, Input, Disturbance, or Trajectory instances from the corresponding input arguments (fields of the corresponding dataclasses) or numpy arrays.
    """

    state_dimension: int
    input_dimension: int
    disturbance_dimension: int

    @classmethod
    @abstractmethod
    def state_from_args(cls, *args) -> StateInterface:
        """
        Create State from args
        """
        pass

    @classmethod
    @abstractmethod
    def input_from_args(cls, *args) -> InputInterface:
        """
        Create Input args
        """
        pass

    @classmethod
    @abstractmethod
    def disturbance_from_args(cls, *args) -> DisturbanceInterface:
        """
        Crate Disturbance args
        """
        pass

    @classmethod
    @abstractmethod
    def state_from_numpy_array(
        cls,
        x_np: np.ndarray,
    ) -> StateInterface:
        """
        Create State from numpy array.
        """
        pass

    @classmethod
    @abstractmethod
    def input_from_numpy_array(cls, u_np: np.ndarray) -> InputInterface:
        """
        Create Input from numpy array.
        """
        pass

    @classmethod
    @abstractmethod
    def disturbance_from_numpy_array(cls, w_np: np.ndarray) -> Union[Any]:
        """
        Create Disturbance from numpy array.
        """
        pass

    @classmethod
    @abstractmethod
    def trajectory_from_points(
        cls,
        trajectory_dict: Union[
            Dict[int, StateInterface],
            Dict[int, InputInterface],
            Dict[int, DisturbanceInterface],
        ],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> TrajectoryInterface:
        """
        Create State, Input, or Disturbance Trajectory from list of points.
        """
        pass

    @classmethod
    @abstractmethod
    def trajectory_from_numpy_array(
        cls,
        traj_np: np.ndarray,
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> TrajectoryInterface:
        """
        Create State, Input, or Disturbance Trajectory from numpy array.
        """
        pass
