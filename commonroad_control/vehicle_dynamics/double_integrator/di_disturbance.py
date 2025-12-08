from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
    DisturbanceInterfaceIndex,
)


@dataclass(frozen=True)
class DIDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbances of the double integrator model.
    """

    dim: int = 4
    position_long: int = 0
    position_lat: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3


@dataclass
class DIDisturbance(DisturbanceInterface):
    """
    Dataclass storing the disturbances of the double integrator model.
    """

    position_long: float = 0.0
    position_lat: float = 0.0
    velocity_long: float = 0.0
    velocity_lat: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: disturbance dimension
        """
        return DIDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[DIDisturbanceIndices.position_long] = self.position_long
        w_np[DIDisturbanceIndices.position_lat] = self.position_lat
        w_np[DIDisturbanceIndices.velocity_long] = self.velocity_long
        w_np[DIDisturbanceIndices.velocity_lat] = self.velocity_lat

        return w_np
