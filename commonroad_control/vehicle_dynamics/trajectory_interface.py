import math
from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Any, Union, Dict, Optional, Tuple, List
import numpy as np

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode


@dataclass
class TrajectoryInterface(ABC):
    # TODO Move some stuff to properties for access control as frozen does not work in our case?
    points: Dict[int, Any]
    delta_t: float
    mode: TrajectoryMode
    t_0: float = 0
    steps: List[int] = None
    t_final: Optional[float] = None
    initial_point: Optional[Any] = None
    final_point: Optional[Any] = None
    dim: Optional[int] = None

    def __post_init__(self):
        self.sanity_check()
        self.dim = self.points[0].dim
        self.initial_point = self.points[min(self.points.keys())]
        self.final_point = self.points[max(self.points.keys())]
        self.steps = sorted(self.points.keys())
        self.t_final = self.t_0 + len(self.points.keys()) * self.delta_t

    def sanity_check(self) -> None:
        """
        Sanity check
        """
        if len(self.points.keys()) == 0:
            raise ValueError(f"states must contain more than 0 values")
        if None in self.points.values():
            raise ValueError(f"states must not contain None")
        d = self.points[0].dim
        for state in self.points.values():
            if state.dim != d:
                raise ValueError("states have varying dimension")

    def convert_to_numpy_array(self, time: List) -> np.array:
        """

        :param time:
        :return:
        """
        traj_np = []
        # TODO: fix time step conversion
        for ii in range(len(time)):
            x_ii = self.get_point_at_time_step(round((time[ii]-self.t_0)/self.delta_t))
            traj_np.append(np.reshape(x_ii.convert_to_array(),(x_ii.dim, 1), order='F'))

        return np.hstack(traj_np)

    def get_point_at_time_step(
            self,
            time_step: int
    ) -> Union[StateInterface, InputInterface, None]:
        """
        Returns the trajectory point at a given time step or None if not existing.
        :param time_step: time step
        :return: StateInterface/InputInterface at step or None if not existing
        """

        return self.points[time_step] if time_step in self.points.keys() else None

    def get_point_before_and_after_time(
            self,
            time: float
    ) -> Tuple[Any, Any, int, int]:
        """
        Finds states before and after a given time steps
        :param time: time
        :return: state_before, state_after, idx_before, idx_after
        """
        if time < self.t_0:
            raise ValueError(f"time {time} is before trajectory start {self.t_0}")
        idx_lower: int = math.floor((time - self.t_0) / self.delta_t)
        idx_upper: int = math.ceil((time - self.t_0) / self.delta_t)

        return self.points[idx_lower], self.points[idx_upper], idx_lower, idx_upper

    def append_point(
            self,
            next_point: Union[StateInterface, InputInterface]) -> None:
        """
        Appends a point to the trajectory.
        :param next_point: point to be appended
        :return:
        """
        if type(next_point) is type(self.final_point):
            self.points[self.steps[-1]+1] = next_point
            self.__post_init__()
        else:
            raise TypeError(f"Expected point of type {type(self.final_point)}, "
                            f"got {type(next_point)}instead")

    @abstractmethod
    def get_interpolated_point_at_time(
            self,
            time: float,
            factory: Any
    ) -> Union[StateInterface, InputInterface]:
        """
        Interpolate trajectory point at given point in time.
        :param time: point in time
        :return: StateInterface/InputInterface
        """
        pass
