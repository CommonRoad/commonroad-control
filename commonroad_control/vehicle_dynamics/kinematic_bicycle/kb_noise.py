from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.full_state_noise_interface import (
    FullStateNoiseInterface,
    FullStateNoiseInterfaceIndex,
)


@dataclass(frozen=True)
class KBNoiseIndices(FullStateNoiseInterfaceIndex):
    """
    Indices of the noise variables.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


@dataclass
class KBNoise(FullStateNoiseInterface):
    """
    Full state noise of the kinematic bicycle model - required for the full state feedback sensor model.
    """

    position_x: float = 0.0
    position_y: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    steering_angle: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: noise dimension
        """
        return KBNoiseIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[KBNoiseIndices.position_x] = self.position_x
        w_np[KBNoiseIndices.position_y] = self.position_y
        w_np[KBNoiseIndices.velocity] = self.velocity
        w_np[KBNoiseIndices.heading] = self.heading
        w_np[KBNoiseIndices.steering_angle] = self.steering_angle

        return w_np
