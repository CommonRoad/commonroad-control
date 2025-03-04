from dataclasses import dataclass
import numpy as np
from typing import Union

from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


@dataclass
class DITrajectory(TrajectoryInterface):
    """
    Dynamic Single Track Trajectory
    """

    def get_interpolated_point_at_time(
            self,
            time: float
    ) -> Union['DIInput', 'DIState']:
        """
        :param time: time at which to interpolate
        :return: interpolated state
        """
        pass
