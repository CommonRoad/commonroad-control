import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.input_interface import InputInterface, InputInterfaceIndex


@dataclass(frozen=True)
class KSTInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs.
    """
    jerk: int = 0
    steering_angle_velocity: int = 1


@dataclass
class KSTInput(InputInterface):
    """
    Control input of kinematic single track.
    """
    dim: int = 2
    jerk: float = None
    steering_angle_velocity: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        u = np.zeros((self.dim,))
        u[KSTInputIndices.jerk] = self.jerk
        u[KSTInputIndices.steering_angle_velocity] = self.steering_angle_velocity

        return u

    def set_values_from_np_array(self, u: np.array) -> None:
        """
        Set values from a given array.
        :param u: input vector - array of dimension (self.dim,)
        """
        if u.size() > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {u}")

        if u.shape[0] != self.dim:
            raise ValueError(f"input should be ({self.dim},) but is {u}")

        self.jerk = u[KSTInputIndices.jerk]
        self.steering_angle_velocity = u[KSTInputIndices.steering_angle_velocity]
