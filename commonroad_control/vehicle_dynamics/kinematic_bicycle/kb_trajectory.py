import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, List, Union

import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_disturbance import (
    KBDisturbance,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.trajectory_interface import (
    TrajectoryInterface,
    TrajectoryMode,
)

if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import (
        KBSIDTFactory,
    )

logger = logging.getLogger(__name__)


@dataclass
class KBTrajectory(TrajectoryInterface):
    """
    Kinematic bicycle model Trajectory.
    """

    def get_point_at_time(
        self, time: float, factory: "KBSIDTFactory"
    ) -> Union["KBState", "KBInput", "KBDisturbance"]:
        """
        Computes a point at a given time by linearly interpolating between the trajectory points at the adjacent
        (discrete) time steps.
        :param time: time at which to interpolate
        :param factory: sidt_factory for instantiating the interpolated point (dataclass object)
        :return: interpolated point - KBState/KBInput/KBDisturbance
        """

        lower_point, upper_point, lower_idx, upper_idx = (
            self.get_point_before_and_after_time(time=time)
        )
        if lower_idx == upper_idx:
            new_point = lower_point
        else:
            alpha = (upper_idx * self.delta_t - time) / self.delta_t
            new_point_array: np.ndarray = (
                1 - alpha
            ) * upper_point.convert_to_array() + alpha * lower_point.convert_to_array()
            if self.mode is TrajectoryMode.State:
                new_point: KBState = factory.state_from_numpy_array(new_point_array)
            elif self.mode is TrajectoryMode.Input:
                new_point: KBInput = factory.input_from_numpy_array(new_point_array)
            elif self.mode is TrajectoryMode.Disturbance:
                new_point: KBDisturbance = factory.disturbance_from_numpy_array(
                    new_point_array
                )
            else:
                logger.error(
                    f"Instantiation of new point not implemented for trajectory mode {self.mode}"
                )
                raise TypeError(
                    f"Instantiation of new point not implemented for trajectory mode {self.mode}"
                )

        return new_point

    def to_cr_dynamic_obstacle(
        self,
        vehicle_width: float,
        vehicle_length: float,
        vehicle_id: int,
    ) -> DynamicObstacle:
        """
        Converts state trajectory to CommonRoad dynamic obstacles for plotting.
        :param vehicle_width: vehicle width
        :param vehicle_length: vehicle length
        :param vehicle_id: vehicle id
        :return: CommonRoad dynamic obstacle
        """

        if self.mode is not TrajectoryMode.State:
            logger.error(
                f"Conversion to dynamic obstacle for plotting not admissible for trajectory points of type {self.mode}"
            )
            raise TypeError(
                f"Conversion to dynamic obstacle for plotting not admissible for trajectory points of type {self.mode}"
            )

        if not self.points:
            logger.error(f"Trajectory.points={self.points} is empty")
            raise ValueError(f"Trajectory.points={self.points} is empty")

        else:
            # convert to CR obstacle
            initial_state: InitialState = self.initial_point.to_cr_initial_state(
                time_step=min(self.points.keys())
            )
            state_list: List[CustomState] = [
                state.to_cr_custom_state(time_step=step)
                for step, state in self.points.items()
            ]

            cr_trajectory = Trajectory(state_list[0].time_step, state_list)
            shape = Rectangle(width=vehicle_width, length=vehicle_length)

            trajectory_prediction = TrajectoryPrediction(
                trajectory=cr_trajectory, shape=shape
            )
            # obstacle generation
            return DynamicObstacle(
                obstacle_id=vehicle_id,
                obstacle_type=ObstacleType.CAR,
                obstacle_shape=shape,
                initial_state=initial_state,
                prediction=trajectory_prediction,
            )
