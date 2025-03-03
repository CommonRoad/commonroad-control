import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


# TODO should be an enum so no initialization?
@dataclass(frozen=True)
class DIStateIndices:
    """
    Indices of the states.
    """
    dim: int = 4
    position_long: int = 0
    position_lat: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3


# TODO: Move to python3.10 and use kw_only dataclass arg?
@dataclass
class DIState(StateInterface):
    """
    State of the kinematic single track model.
    """
    dim: int = DIStateIndices.dim
    position_long: float = None
    position_lat: float = None
    velocity_long: float = None
    velocity_lat: float = None

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """
        x_np = np.zeros((self.dim,))
        x_np[DIStateIndices.position_long] = self.position_long
        x_np[DIStateIndices.position_lat] = self.position_lat
        x_np[DIStateIndices.velocity_long] = self.velocity_long
        x_np[DIStateIndices.velocity_lat] = self.velocity_lat

        return x_np
