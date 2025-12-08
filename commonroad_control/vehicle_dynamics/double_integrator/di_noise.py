from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.full_state_noise_interface import (
    FullStateNoiseInterface,
    FullStateNoiseInterfaceIndex,
)


@dataclass(frozen=True)
class DINoiseIndices(FullStateNoiseInterfaceIndex):
    """
    Indices of the noise variables.
    """

    dim: int = 4
    position_long: int = 0
    position_lat: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3


@dataclass
class DINoise(FullStateNoiseInterface):
    """
    Full state noise of the double integrator model - required for the full state feedback sensor model.
    """

    position_long: float = 0.0
    position_lat: float = 0.0
    velocity_long: float = 0.0
    velocity_lat: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: noise dimension
        """
        return DINoiseIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        y_np = np.zeros((self.dim,))
        y_np[DINoiseIndices.position_long] = self.position_long
        y_np[DINoiseIndices.position_lat] = self.position_lat
        y_np[DINoiseIndices.velocity_long] = self.velocity_long
        y_np[DINoiseIndices.velocity_lat] = self.velocity_lat

        return y_np
