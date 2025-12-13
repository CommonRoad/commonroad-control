from __future__ import annotations

import logging
import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple, Union

import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode, gt_tol, lt_tol

if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
        StateInputDisturbanceTrajectoryFactoryInterface,
    )

logger = logging.getLogger(__name__)


@dataclass
class TrajectoryInterface(ABC):
    """
    Interface for State/Input/Disturbance trajectories of a given vehicle model.
    Trajectory points are stored as a dict of points and assumed to be sampled at constant rate of 1/delta_t.
    """

    points: Dict[int, Any]  # dict of points
    delta_t: float  # sampling time
    mode: TrajectoryMode  # state/input/disturbance trajectory
    t_0: float = 0.0  # initial time
    steps: List[int] = None  # time steps of the trajectory points
    t_final: Optional[float] = None  # final time
    initial_point: Union[StateInterface, InputInterface] = None
    final_point: Union[StateInterface, InputInterface] = None
    dim: Optional[int] = None  # dimension of points

    def __post_init__(self):
        self.sanity_check()
        self.dim = self.points[0].dim
        self.initial_point = self.points[min(self.points.keys())]
        self.final_point = self.points[max(self.points.keys())]
        self.steps = sorted(self.points.keys())
        self.t_final = self.t_0 + max(self.points.keys()) * self.delta_t

    def sanity_check(self) -> None:
        """
        Sanity check
        """
        if len(self.points.keys()) == 0:
            logger.error("Dict of points must contain more than 0 values.")
            raise ValueError("Dict of points must contain more than 0 values")
        if None in self.points.values():
            logger.error("Points must not contain None")
            raise ValueError("Points must not contain None")
        if 0 != min(self.points.keys()):
            logger.error("Time step of initial points must be 0.")
            raise ValueError("Time step of initial point must be 0.")
        initial_point = self.points[0]
        for point in self.points.values():
            if type(point) is not type(initial_point):
                logger.error("Type of trajectory points is not unique.")
                raise TypeError("Type of trajectory points is not unique.")

    def convert_to_numpy_array(
        self,
        time: List[float],
        linear_interpolate: bool = False,
        sidt_factory: Optional[
            "StateInputDisturbanceTrajectoryFactoryInterface"
        ] = None,
    ) -> np.ndarray:
        """
        Extracts the trajectory points at given points in time and stores the point variables in an array.
        By default, the to-be-extracted points are approximated by the point at the closest discrete point in time of the trajectory.
        If a more accurate result is desired, the to-be-extracted points can be approximated using linear interpolation between trajectory points at adjacent time steps (see input argument: linear_interpolate).
        :param time: desired points in time for sampling the trajectory - list of floats
        :param linear_interpolate: if true, compute to-be-extracted points using linear interpolation; otherwise, approximate by the closest trajectory point
        :param sidt_factory: factory for generating points, required if linear_interpolate=True - StateInputDisturbanceTrajectoryFactoryInterface
        :return: (approximation of) desired trajectory points - array of dimension (dim, len(time))
        """

        traj_np = []
        for ii in range(len(time)):
            if linear_interpolate:
                y_ii = self.get_point_at_time(time=time[ii], factory=sidt_factory)
            else:
                y_ii = self.get_point_at_time_step(
                    time_step=round((time[ii] - self.t_0) / self.delta_t)
                )
            traj_np.append(
                np.reshape(y_ii.convert_to_array(), (y_ii.dim, 1), order="F")
            )

        return np.hstack(traj_np)

    def get_point_at_time_step(
        self, time_step: int
    ) -> Union[StateInterface, InputInterface, None]:
        """
        Returns the trajectory point at a given (discrete) time step or None if not existing.
        :param time_step: time step - int
        :return: StateInterface/InputInterface/DisturbanceInterface at step or None if not existing
        """

        return self.points[time_step] if time_step in self.points.keys() else None

    def get_point_before_and_after_time(self, time: float) -> Tuple[Any, Any, int, int]:
        """
        Finds trajectory points at discrete time steps before and after a given point in time.
        :param time: point in time - float
        :return: point before time, point after time, time step before, time step after
        """
        if lt_tol(time, self.t_0):
            logger.error(f"Time {time} is before trajectory initial time {self.t_0}")
            raise ValueError(
                f"Time {time} is before trajectory initial time {self.t_0}"
            )
        if gt_tol(time, self.t_final):
            logger.error(f"Time {time} is after trajectory final time {self.t_final}")
            raise ValueError(
                f"Time {time} is after trajectory final time {self.t_final}"
            )
        idx_lower: int = min(
            math.floor((time - self.t_0) / self.delta_t), max(self.points.keys())
        )
        idx_upper: int = min(
            math.ceil((time - self.t_0) / self.delta_t), max(self.points.keys())
        )

        return self.points[idx_lower], self.points[idx_upper], idx_lower, idx_upper

    def append_point(
        self, next_point: Union[StateInterface, InputInterface, DisturbanceInterface]
    ) -> None:
        """
        Appends a point to the trajectory.
        :param next_point: point to be appended
        """
        if type(next_point) is type(self.final_point):
            self.points[self.steps[-1] + 1] = next_point
            self.__post_init__()
        else:
            logger.error(
                f"Expected point of type {type(self.final_point)}, "
                f"got {type(next_point)}instead"
            )
            raise TypeError(
                f"Expected point of type {type(self.final_point)}, "
                f"got {type(next_point)}instead"
            )

    @abstractmethod
    def get_point_at_time(
        self, time: float, factory: "StateInputDisturbanceTrajectoryFactoryInterface"
    ) -> Union[StateInterface, InputInterface, DisturbanceInterface]:
        """
        Computes a point at a given time by linearly interpolating between the trajectory points at the adjacent
        (discrete) time steps.
        :param time: time at which to interpolate
        :param factory: sidt_factory for instantiating the interpolated point (dataclass object)
        :return: StateInterface/InputInterface/DisturbanceInterface
        """
        pass
