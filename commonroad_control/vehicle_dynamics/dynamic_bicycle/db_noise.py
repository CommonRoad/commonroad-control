from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.full_state_noise_interface import (
    FullStateNoiseInterface,
    FullStateNoiseInterfaceIndex,
)


@dataclass(frozen=True)
class DBNoiseIndices(FullStateNoiseInterfaceIndex):
    """
    Indices of the noise variables.
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
class DBNoise(FullStateNoiseInterface):
    """
    Full state noise of the dynamic bicycle model - required for the full state feedback sensor model.
    """

    position_x: float = 0.0
    position_y: float = 0.0
    velocity_long: float = 0.0
    velocity_lat: float = 0.0
    heading: float = 0.0
    yaw_rate: float = 0.0
    steering_angle: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: noise dimension
        """
        return DBNoiseIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        y_np = np.zeros((self.dim,))
        y_np[DBNoiseIndices.position_x] = self.position_x
        y_np[DBNoiseIndices.position_y] = self.position_y
        y_np[DBNoiseIndices.velocity_long] = self.velocity_long
        y_np[DBNoiseIndices.velocity_lat] = self.velocity_lat
        y_np[DBNoiseIndices.heading] = self.heading
        y_np[DBNoiseIndices.yaw_rate] = self.yaw_rate
        y_np[DBNoiseIndices.steering_angle] = self.steering_angle

        return y_np
