from dataclasses import dataclass
import numpy as np
from typing import Union

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface, TrajectoryMode

from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory

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

    def get_point_at_time(
            self,
            time: float,
            factory: 'DISITFactory'
    ) -> Union['DIState', 'DIInput']:
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
            new_point: Union[DIState,DIInput] = (
                factory.state_from_numpy_array(new_point_array)) if self.mode is TrajectoryMode.State \
                else factory.input_from_numpy_array(new_point_array)
        return new_point
