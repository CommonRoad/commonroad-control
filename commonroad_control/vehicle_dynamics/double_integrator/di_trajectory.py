import logging
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.trajectory_interface import (
    TrajectoryInterface,
)

logger = logging.getLogger(__name__)


@dataclass
class DITrajectory(TrajectoryInterface):
    """
    Double integrator model Trajectory
    """

    def to_cr_dynamic_obstacle(
        self,
        vehicle_width: float,
        vehicle_length: float,
        vehicle_id: int,
    ):
        """
        Converts state trajectory to CommonRoad dynamic obstacles for plotting
        :param vehicle_width: vehicle width
        :param vehicle_length: vehicle length
        :param vehicle_id: vehicle id
        :return: CommonRoad dynamic obstacle
        """
        raise NotImplementedError("to_cr_dynamic_obstacle() has not been implemented yet.")
