import math
from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Any, Union, Dict, Optional, Tuple, List
import numpy as np

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.sidt_factory_interface import StateInputDisturbanceTrajectoryFactoryInterface


@dataclass
class TrajectoryInterface(ABC):
    # TODO Move some stuff to properties for access control as frozen does not work in our case?
    points: Dict[int, Any]
    delta_t: float
    mode: TrajectoryMode
    t_0: float = 0
    steps: List[int] = None
    t_final: Optional[float] = None
    initial_point: Union[StateInterface, InputInterface] = None
    final_point: Union[StateInterface, InputInterface] = None
    dim: Optional[int] = None

    def __post_init__(self):
        self.sanity_check()
        self.dim = self.points[0].dim
        self.initial_point = self.points[min(self.points.keys())]
        self.final_point = self.points[max(self.points.keys())]
        #TODO: check steps - 0 must be contained for initial point
        self.steps = sorted(self.points.keys())
        self.t_final = self.t_0 + max(self.points.keys()) * self.delta_t

    def sanity_check(self) -> None:
        """
        Sanity check
        """
        if len(self.points.keys()) == 0:
            raise ValueError(f"states must contain more than 0 values")
        if None in self.points.values():
            raise ValueError(f"states must not contain None")
        initial_point = self.points[0]
        for point in self.points.values():
            if type(point) is not type(initial_point):
                raise TypeError(f"Type of trajectory points is not unique.")

    def convert_to_numpy_array(self, time: List[float]) -> np.ndarray:
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

    # TODO: remove if not required
    def get_point_component_trajectory(self,
                                       time: List[float],
                                       comp_name: str) -> List[float]:
        """
        Returns the trajectory of a single component of the point.
        :param time: list of points in time, will be rounded to the closest time step.
        :param comp_name: component of the state vector, e.g. velocity or heading
        :return: list of component values at provided time steps
        """

        if not hasattr(self.initial_point,comp_name):
            raise ValueError(f"Trajectory points do not have attribute {comp_name}.")

        traj = []
        for kk in range(len(time)):
            tmp_x = self.get_point_at_time_step(round((time[kk] - self.t_0) / self.delta_t))
            traj.append(getattr(tmp_x, comp_name))

        return traj

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
        idx_lower: int = min(math.floor((time - self.t_0) / self.delta_t), max(self.points.keys()))
        idx_upper: int = min(math.ceil((time - self.t_0) / self.delta_t), max(self.points.keys()))

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
    def get_point_at_time(
            self,
            time: float,
            factory: 'StateInputDisturbanceTrajectoryFactoryInterface'
    ) -> Union[StateInterface, InputInterface]:
        """
        Computes a point at a given time by linearly interpolating between the trajectory points at the adjacent
        (discrete) time steps.
        :param time: time at which to interpolate
        :param factory: sit_factory for instantiating the interpolated point (dataclass object)
        :return: StateInterface/InputInterface
        """
        pass
