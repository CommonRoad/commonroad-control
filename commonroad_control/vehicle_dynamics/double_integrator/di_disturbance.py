from dataclasses import dataclass

import numpy as np

from commonroad_control.vehicle_dynamics.disturbance_interface import (
    DisturbanceInterface,
    DisturbanceInterfaceIndex,
)


# TODO should be an enum so no initialization?
@dataclass(frozen=True)
class DIDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbance.
    """

    dim: int = 4
    position_long: int = 0
    position_lat: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3


# TODO: Move to python3.10 and use kw_only dataclass arg?
@dataclass
class DIDisturbance(DisturbanceInterface):
    """
    Disturbance of the double integrator model.
    """

    position_long: float = None
    position_lat: float = None
    velocity_long: float = None
    velocity_lat: float = None

    @property
    def dim(self):
        return DIDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[DIDisturbanceIndices.position_long] = self.position_long
        w_np[DIDisturbanceIndices.position_lat] = self.position_lat
        w_np[DIDisturbanceIndices.velocity_long] = self.velocity_long
        w_np[DIDisturbanceIndices.velocity_lat] = self.velocity_lat

        return w_np
