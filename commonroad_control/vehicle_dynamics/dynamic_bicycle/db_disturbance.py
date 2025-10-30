import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.disturbance_interface import DisturbanceInterface, DisturbanceInterfaceIndex


@dataclass(frozen=True)
class DBDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbances.
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
class DBDisturbance(DisturbanceInterface):
    """
    Disturbance of the dynamic bicycle model.
    """
    position_x: float = None
    position_y: float = None
    velocity_long: float = None
    velocity_lat: float = None
    heading: float = None
    yaw_rate: float = None
    steering_angle: float = None

    @property
    def dim(self):
        return DBDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        w_np = np.zeros((self.dim,))
        w_np[DBDisturbanceIndices.position_x] = self.position_x
        w_np[DBDisturbanceIndices.position_y] = self.position_y
        w_np[DBDisturbanceIndices.velocity_long] = self.velocity_long
        w_np[DBDisturbanceIndices.velocity_lat] = self.velocity_lat
        w_np[DBDisturbanceIndices.heading] = self.heading
        w_np[DBDisturbanceIndices.yaw_rate] = self.yaw_rate
        w_np[DBDisturbanceIndices.steering_angle] = self.steering_angle

        return w_np