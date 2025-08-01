from pathlib import Path
import os

import matplotlib.pyplot as plt

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface

from typing import Optional, List, Union




def visualize_ff_and_fb_inputs(
    feed_forward_inputs: Union[TrajectoryInterface, KSTTrajectory],
    feedback_inputs: Union[TrajectoryInterface, KSTTrajectory],
    time_steps_ff: List[int],
    time_steps_fb: List[int],
    state_dim: int,
    scenario_name: str,
    state_names: Optional[List[str]] = None,
    save_img: bool = False,
    save_path: Union[str, Path] = None,
) -> None:
    pass