from dataclasses import dataclass

import numpy as np
from commonroad.scenario.state import CustomState, InitialState

from commonroad_control.vehicle_dynamics.state_interface import (
    StateInterface,
    StateInterfaceIndex,
)


@dataclass(frozen=True)
class KBStateIndices(StateInterfaceIndex):
    """
    Indices of the states of the kinematic bicycle model.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


@dataclass
class KBState(StateInterface):
    """
    Dataclass storing the states of the kinematic bicycle model.
    """

    position_x: float = None
    position_y: float = None
    velocity: float = None
    heading: float = None
    steering_angle: float = None

    @property
    def dim(self) -> int:
        """
        :return: state dimension
        """
        return KBStateIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        x_np = np.zeros((self.dim,))
        x_np[KBStateIndices.position_x] = self.position_x
        x_np[KBStateIndices.position_y] = self.position_y
        x_np[KBStateIndices.velocity] = self.velocity
        x_np[KBStateIndices.heading] = self.heading
        x_np[KBStateIndices.steering_angle] = self.steering_angle

        return x_np

    def to_cr_initial_state(self, time_step: int) -> InitialState:
        """
        Convert to CommonRoad initial state
        :param time_step: time step
        :return: CommonRoad InitialState
        """
        return InitialState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=self.velocity,
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=0,
            time_step=time_step,
        )

    def to_cr_custom_state(self, time_step: int) -> CustomState:
        """
        Convert to CommonRoad custom state
        :param time_step: time step -int
        :return: CommonRoad custom state
        """
        return CustomState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=self.velocity,
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=0,
            time_step=time_step,
        )
