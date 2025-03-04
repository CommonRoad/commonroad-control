import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface, StateInterfaceIndex


@dataclass(frozen=True)
class DBStateIndices(StateInterfaceIndex):
    """
    Indices of the states.
    """
    dim: int = 7
    position_x: int = 0
    position_y: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3
    heading: int = 4
    yaw_rate: int = 5
    steering_angle = 6


@dataclass
class DBState(StateInterface):
    """
    State of the dynamic bicycle model
    """
    dim: int = DBStateIndices.dim
    position_x: float = None
    position_y: float = None
    velocity_long: float = None
    velocity_lat: float = None
    heading: float = None
    yaw_rate: float = None
    steering_angle: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """

        x = np.zeros((self.dim,))
        x[DBStateIndices.position_x] = self.position_x
        x[DBStateIndices.position_y] = self.position_y
        x[DBStateIndices.velocity_long] = self.velocity_long
        x[DBStateIndices.velocity_lat] = self.velocity_lat
        x[DBStateIndices.heading] = self.heading
        x[DBStateIndices.yaw_rate] = self.yaw_rate
        x[DBStateIndices.steering_angle] = self.steering_angle

        return x
