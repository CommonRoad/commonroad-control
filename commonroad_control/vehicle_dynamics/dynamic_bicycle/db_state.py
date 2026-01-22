from dataclasses import dataclass

import numpy as np
from commonroad.scenario.state import CustomState, InitialState

from commonroad_control.util.conversion_util import (
    compute_slip_angle_from_velocity_components,
    compute_velocity_from_components,
)
from commonroad_control.vehicle_dynamics.state_interface import (
    StateInterface,
    StateInterfaceIndex,
)


@dataclass(frozen=True)
class DBStateIndices(StateInterfaceIndex):
    """
    Indices of the states of the dynamic bicycle model.
    """

    dim: int = 7
    position_x: int = 0
    position_y: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3
    heading: int = 4
    yaw_rate: int = 5
    steering_angle: int = 6


@dataclass
class DBState(StateInterface):
    """
    Dataclass storing the states of the dynamic bicycle model.
    """

    position_x: float = None
    position_y: float = None
    velocity_long: float = None
    velocity_lat: float = None
    heading: float = None
    yaw_rate: float = None
    steering_angle: float = None

    @property
    def dim(self) -> int:
        """
        :return: state dimension
        """
        return DBStateIndices.dim

    @property
    def velocity(self) -> float:
        """
        :return: absolute value of velocity of the vehicle
        """
        return compute_velocity_from_components(v_long=self.velocity_long, v_lat=self.velocity_lat)

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        x_np = np.zeros((self.dim,))
        x_np[DBStateIndices.position_x] = self.position_x
        x_np[DBStateIndices.position_y] = self.position_y
        x_np[DBStateIndices.velocity_long] = self.velocity_long
        x_np[DBStateIndices.velocity_lat] = self.velocity_lat
        x_np[DBStateIndices.heading] = self.heading
        x_np[DBStateIndices.yaw_rate] = self.yaw_rate
        x_np[DBStateIndices.steering_angle] = self.steering_angle

        return x_np

    def to_cr_initial_state(self, time_step: int) -> InitialState:
        """
        Convert to CommonRoad initial state
        :param time_step: time step - int
        :return: CommonRoad InitialState
        """
        return InitialState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=compute_velocity_from_components(v_long=self.velocity_long, v_lat=self.velocity_lat),
            orientation=self.heading,
            acceleration=0,
            yaw_rate=self.yaw_rate,
            slip_angle=compute_slip_angle_from_velocity_components(self.velocity_long, self.velocity_lat),
            time_step=time_step,
        )

    def to_cr_custom_state(self, time_step: int) -> CustomState:
        """
        Convert to CommonRoad custom state
        :param time_step: time step - int
        :return: CommonRoad custom state
        """
        return CustomState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=compute_velocity_from_components(v_long=self.velocity_long, v_lat=self.velocity_lat),
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=compute_slip_angle_from_velocity_components(self.velocity_long, self.velocity_lat),
            time_step=time_step,
        )
