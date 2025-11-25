from dataclasses import dataclass
from typing import Union, List
import numpy as np
import logging

from commonroad.geometry.shape import Rectangle
from commonroad.prediction.prediction import TrajectoryPrediction, Trajectory
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.state import InitialState, CustomState

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface, TrajectoryMode

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory

logger = logging.getLogger(__name__)

@dataclass
class DBTrajectory(TrajectoryInterface):
    """
    Dynamic Single Track Trajectory
    """

    def get_point_at_time(
            self,
            time: float,
            factory: 'DBSIDTFactory'
    ) -> Union['DBState', 'DBInput']:
        """
        Computes a point at a given time by linearly interpolating between the trajectory points at the adjacent
        (discrete) time steps.
        :param time: time at which to interpolate
        :param factory: sit_factory for instantiating the interpolated point (dataclass object)
        :return: interpolated point
        """

        lower_point, upper_point, lower_idx, upper_idx = self.get_point_before_and_after_time(
            time=time
        )
        if lower_idx == upper_idx:
            new_point = lower_point
        else:
            alpha = (upper_idx*self.delta_t - time) / self.delta_t
            new_point_array: np.ndarray = (
                    alpha*upper_point.convert_to_array() + (1-alpha)*lower_point.convert_to_array()
            )
            new_point: Union[DBState,DBInput] = (
                factory.state_from_numpy_array(new_point_array)) if self.mode is TrajectoryMode.State \
                else factory.input_from_numpy_array(new_point_array)
        return new_point


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
            logger.error(f"Trajectory.points={self.points}  is empty")
            raise ValueError(f"Trajectory.points={self.points}  is empty")

        else:
            # convert to CR obstacle
            initial_state: InitialState = self.initial_point.to_cr_initial_state(time_step=min(self.points.keys()))
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