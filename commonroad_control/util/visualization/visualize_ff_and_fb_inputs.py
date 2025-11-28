import logging
from pathlib import Path
from typing import List, Optional, Union

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import (
    KBTrajectory,
)
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface

logger = logging.getLogger(__name__)


def visualize_ff_and_fb_inputs(
    feed_forward_inputs: Union[TrajectoryInterface, KBTrajectory],
    feedback_inputs: Union[TrajectoryInterface, KBTrajectory],
    time_steps_ff: List[int],
    time_steps_fb: List[int],
    state_dim: int,
    scenario_name: str,
    state_names: Optional[List[str]] = None,
    save_img: bool = False,
    save_path: Union[str, Path] = None,
) -> None:
    logger.error("Not implemented")
    raise NotImplementedError("Not implemented")
