import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface, StateInterfaceIndex


@dataclass(frozen=True)
class DBStateIndices(StateInterfaceIndex):
    """
    Indices of the states.
    """
    position_x: int = 0
    position_y: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3
    acceleration: int = 4
    heading: int = 5
    yaw_rate: int = 6
    steering_angle = 7

    def __post_init__(self):
        super().__init__()


@dataclass
class DBState(StateInterface):
    """
    State of the dynamic bicycle model
    """
    dim: int = 8
    position_x: float = None
    position_y: float = None
    velocity_long: float = None
    velocity_lat: float = None
    acceleration: float = None
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
        x[DBStateIndices.acceleration] = self.acceleration
        x[DBStateIndices.heading] = self.heading
        x[DBStateIndices.yaw_rate] = self.yaw_rate
        x[DBStateIndices.steering_angle] = self.steering_angle

        return x

    def set_values_from_np_array(
            self,
            x: np.array
    ) -> None:
        """
        Set values of class from a given array.
        :param x: state vector - array of dimension (dim,)
        """

        self.position_x = x[DBStateIndices.position_x]
        self.position_y = x[DBStateIndices.position_y]
        self.velocity_long = x[DBStateIndices.velocity_long]
        self.velocity_lat = x[DBStateIndices.velocity_lat]
        self.acceleration = x[DBStateIndices.acceleration]
        self.heading = x[DBStateIndices.heading]
        self.yaw_rate = x[DBStateIndices.yaw_rate]
        self.steering_angle = x[DBStateIndices.steering_angle]
