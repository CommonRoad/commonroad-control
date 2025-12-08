from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
    DisturbanceInterfaceIndex,
)


@dataclass(frozen=True)
class KBDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbances of the kinematic bicycle model.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


@dataclass
class KBDisturbance(DisturbanceInterface):
    """
    Dataclass storing the disturbances of the kinematic bicycle model.
    """

    position_x: float = 0.0
    position_y: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    steering_angle: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: disturbance dimension
        """
        return KBDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[KBDisturbanceIndices.position_x] = self.position_x
        w_np[KBDisturbanceIndices.position_y] = self.position_y
        w_np[KBDisturbanceIndices.velocity] = self.velocity
        w_np[KBDisturbanceIndices.heading] = self.heading
        w_np[KBDisturbanceIndices.steering_angle] = self.steering_angle

        return w_np
