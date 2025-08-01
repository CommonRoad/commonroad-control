from dataclasses import dataclass
from typing import Union, List

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, CustomState

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


@dataclass
class DBTrajectory(TrajectoryInterface):
    """
    Dynamic Single Track Trajectory
    """

    def get_interpolated_point_at_time(
            self,
            time: float
    ) -> Union['DBInput', 'DBState']:
        """
        :param time: time at which to interpolate
        :return: interpolated state
        """
        pass


    def to_cr_dynamic_obstacle(
        self,
        vehicle_width: float,
        vehicle_length: float,
        vehicle_id: int,
    ) -> DynamicObstacle:
        """
        Converts trajectory cr dynamic obstacle for plotting
        :param vehicle_width: vehicle width
        :param vehicle_length: vehicle length
        :param vehicle_id: vehicle id
        :return: cr dynamic obstacle
        """

        if not self.points:
            raise ValueError(f"Trajectory.points={self.points}  is empty")

        else:
            # convert to CR obstacle
            initial_state: InitialState = self.initial_state.to_cr_initial_state(time_step=min(self.points.keys()))
            state_list: List[CustomState] = [
                state.to_cr_custom_state(time_step=step) for step, state in self.points.items()
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