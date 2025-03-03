from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from typing import Dict, List, Tuple, Union, Any, Optional, Literal

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


@dataclass
class KSTTrajectory(TrajectoryInterface):
    """
    Kinematic Single Track Trajectory
    """

    def get_interpolated_point_at_time(
            self,
            time: float
    ) -> Union['KSTState', 'KSTInput']:
        """
        :param time: time at which to interpolate
        :return: interpolated state
        """
        # TODO Test
        # TODO Handle angle jumps
        lower_state, upper_state, lower_idx, upper_idx = self.get_states_before_and_after_time(time=time)
        if lower_idx == upper_idx:
            new_state = lower_state
        else:
            new_state_array: np.ndarray = (
                    ((upper_state.convert_to_array() - lower_state.convert_to_array()) / self.delta_t)
                    * (upper_idx*self.delta_t - time)
                    + lower_state.convert_to_array()
            )
            new_state: KSTState = (
                KSTSITFactory().state_from_numpy_array(new_state_array)) if self.mode == 'state' \
                else KSTSITFactory().input_from_numpy_array(new_state_array)
        return new_state