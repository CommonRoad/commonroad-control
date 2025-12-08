from dataclasses import dataclass

import numpy as np
from commonroad.scenario.state import CustomState, InitialState

from commonroad_control.vehicle_dynamics.state_interface import (
    StateInterface,
    StateInterfaceIndex,
)


@dataclass(frozen=True)
class DIStateIndices(StateInterfaceIndex):
    """
    Indices of the states of the double integrator model.
    """

    dim: int = 4
    position_long: int = 0
    position_lat: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3


@dataclass
class DIState(StateInterface):
    """
    Dataclass storing the states of the double integrator model.
    """

    position_long: float = None
    position_lat: float = None
    velocity_long: float = None
    velocity_lat: float = None

    @property
    def dim(self) -> int:
        """
        :return: state dimension
        """
        return DIStateIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        x_np = np.zeros((self.dim,))
        x_np[DIStateIndices.position_long] = self.position_long
        x_np[DIStateIndices.position_lat] = self.position_lat
        x_np[DIStateIndices.velocity_long] = self.velocity_long
        x_np[DIStateIndices.velocity_lat] = self.velocity_lat

        return x_np

    def to_cr_initial_state(self, time_step: int) -> InitialState:
        """
        Convert to CommonRoad initial state
        :param time_step: time step - int
        :return: CommonRoad InitialState
        """
        raise NotImplementedError("to_cr_initial_state() has not been implemented yet.")

    def to_cr_custom_state(self, time_step: int) -> CustomState:
        """
        Convert to CommonRoad custom state
        :param time_step: time step - int
        :return: CommonRoad custom state
        """
        raise NotImplementedError("to_cr_custom_state() has not been implemented yet.")
