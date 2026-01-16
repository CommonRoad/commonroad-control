import logging
from dataclasses import dataclass
from typing import List

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import Trajectory, TrajectoryPrediction
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import CustomState, InitialState

from commonroad_control.vehicle_dynamics.trajectory_interface import (
    TrajectoryInterface,
    TrajectoryMode,
)

logger = logging.getLogger(__name__)


@dataclass
class KBTrajectory(TrajectoryInterface):
    """
    Kinematic bicycle model Trajectory.
    """

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
