import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


@dataclass(frozen=True)
class KSTStateIndices:
    """
    Indices of the states.
    """
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    acceleration: int = 3
    heading: int = 4
    steering_angle = 5

    def __post_init__(self):
        super().__init__()


@dataclass
class KSTState(StateInterface):
    """
    State of the kinematic single track model.
    """
    dim: int = 6
    position_x: float = None
    position_y: float = None
    velocity: float = None
    acceleration: float = None
    heading: float = None
    steering_angle: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """

        x = np.zeros((self.dim,))
        x[KSTStateIndices.position_x] = self.position_x
        x[KSTStateIndices.position_y] = self.position_y
        x[KSTStateIndices.velocity] = self.velocity
        x[KSTStateIndices.acceleration] = self.acceleration
        x[KSTStateIndices.heading] = self.heading
        x[KSTStateIndices.steering_angle] = self.steering_angle

        return x

    def set_values_from_np_array(self, x: np.array) -> None:
        """
        Set values of class from a given array.
        :param x: state vector - array of dimension (dim,)
        """
        if x.size > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {x}")

        if x.shape[0] != self.dim:
            raise ValueError(f"input should be ({self.dim},) but is {x}")

        self.position_x = x[KSTStateIndices.position_x]
        self.position_y = x[KSTStateIndices.position_y]
        self.velocity = x[KSTStateIndices.velocity]
        self.acceleration = x[KSTStateIndices.acceleration]
        self.heading = x[KSTStateIndices.heading]
        self.steering_angle = x[KSTStateIndices.steering_angle]
