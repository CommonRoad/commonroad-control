import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.disturbance_interface import DisturbanceInterface, DisturbanceInterfaceIndex


# TODO should be an enum so no initialization?
@dataclass(frozen=True)
class KBDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbances.
    """
    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


# TODO: Move to python3.10 and use kw_only dataclass arg?
@dataclass
class KBDisturbance(DisturbanceInterface):
    """
    Disturbance of the kinematic bicycle model.
    """
    position_x: float = None
    position_y: float = None
    velocity: float = None
    heading: float = None
    steering_angle: float = None

    @property
    def dim(self):
        return KBDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[KBDisturbanceIndices.position_x] = self.position_x
        w_np[KBDisturbanceIndices.position_y] = self.position_y
        w_np[KBDisturbanceIndices.velocity] = self.velocity
        w_np[KBDisturbanceIndices.heading] = self.heading
        w_np[KBDisturbanceIndices.steering_angle] = self.steering_angle

        return w_np