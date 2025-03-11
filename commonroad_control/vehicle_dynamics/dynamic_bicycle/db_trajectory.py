from dataclasses import dataclass
from typing import Union

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