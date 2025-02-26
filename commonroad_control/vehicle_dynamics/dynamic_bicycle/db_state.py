import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


@dataclass(frozen=True)
class DSTStateIndices:
    """
    Indices of the states.
    """
    position_x: int = 0
    position_y: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3
    heading: int = 4
    yaw_rate: int = 5
    steering_angle = 6


@dataclass
class DSTState(StateInterface):
    """
    State of the dynamic bicycle model
    """
    dim: int = 8
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
        :return: np.ndarray of dimension (dim,1)
        """

        x = np.zeros((self.dim,))
        x[DSTStateIndices.position_x] = self.position_x
        x[DSTStateIndices.position_y] = self.position_y
        x[DSTStateIndices.velocity_long] = self.velocity_long
        x[DSTStateIndices.velocity_lat] = self.velocity_lat
        x[DSTStateIndices.heading] = self.heading
        x[DSTStateIndices.yaw_rate] = self.yaw_rate
        x[DSTStateIndices.steering_angle] = self.steering_angle

        return x

    def set_values_from_np_array(
            self,
            x: np.array
    ) -> None:
        """
        Set values of class from a given array.
        :param x: state vector - array of dimension (dim,1)
        """
        self.position_x = x[DSTStateIndices.position_x]
        self.position_y = x[DSTStateIndices.position_y]
        self.velocity_long = x[DSTStateIndices.velocity_long]
        self.velocity_lat = x[DSTStateIndices.velocity_lat]
        self.heading = x[DSTStateIndices.heading]
        self.yaw_rate = x[DSTStateIndices.yaw_rate]
        self.steering_angle = x[DSTStateIndices.steering_angle]
