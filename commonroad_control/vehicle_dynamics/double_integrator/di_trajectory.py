import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Union

import numpy as np

from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import (
    DIDisturbance,
)
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState
from commonroad_control.vehicle_dynamics.trajectory_interface import (
    TrajectoryInterface,
    TrajectoryMode,
)

if TYPE_CHECKING:
    from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import (
        DISIDTFactory,
    )

logger = logging.getLogger(__name__)


@dataclass
class DITrajectory(TrajectoryInterface):
    """
    Double integrator model Trajectory
    """

    def get_point_at_time(
        self, time: float, factory: "DISIDTFactory"
    ) -> Union["DIState", "DIInput", "DIDisturbance"]:
        """
        Computes a point at a given point in time by linearly interpolating between the trajectory points at the adjacent
        (discrete) time steps.
        :param time: time at which to interpolate - float
        :param factory: sidt_factory for instantiating the interpolated point (dataclass object)
        :return: interpolated point - DIState/DIInput/DIDisturbance
        """

        lower_point, upper_point, lower_idx, upper_idx = (
            self.get_point_before_and_after_time(time=time)
        )
        if lower_idx == upper_idx:
            new_point = lower_point
        else:
            alpha = (upper_idx * self.delta_t - time) / self.delta_t
            new_point_array: np.ndarray = (
                1 - alpha
            ) * upper_point.convert_to_array() + alpha * lower_point.convert_to_array()
            if self.mode is TrajectoryMode.State:
                new_point: DIState = factory.state_from_numpy_array(new_point_array)
            elif self.mode is TrajectoryMode.Input:
                new_point: DIInput = factory.input_from_numpy_array(new_point_array)
            elif self.mode is TrajectoryMode.Disturbance:
                new_point: DIDisturbance = factory.disturbance_from_numpy_array(
                    new_point_array
                )
            else:
                logger.error(
                    f"Instantiation of new point not implemented for trajectory mode {self.mode}"
                )
                raise TypeError(
                    f"Instantiation of new point not implemented for trajectory mode {self.mode}"
                )
        return new_point

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
        raise NotImplementedError(
            "to_cr_dynamic_obstacle() has not been implemented yet."
        )
