from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.input_interface import (
    InputInterface,
    InputInterfaceIndex,
)


@dataclass(frozen=True)
class KBInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs of the kinematic bicycle model.
    """

    dim: int = 2
    acceleration: int = 0
    steering_angle_velocity: int = 1


@dataclass()
class KBInput(InputInterface):
    """
    Dataclass storing the control input of the kinematic bicycle model.
    """

    acceleration: float = None
    steering_angle_velocity: float = None

    @property
    def dim(self) -> int:
        """
        :return: control input dimension
        """
        return KBInputIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        u_np = np.zeros((self.dim,))
        u_np[KBInputIndices.acceleration] = self.acceleration
        u_np[KBInputIndices.steering_angle_velocity] = self.steering_angle_velocity

        return u_np
