import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.input_interface import InputInterface, InputInterfaceIndex


@dataclass(frozen=True)
class DBInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs.
    """
    dim: int = 2
    jerk: int = 0
    steering_angle_velocity: int = 1


@dataclass
class DBInput(InputInterface):
    """
    Control input of the dynamic bicycle model.
    """
    dim: int = DBInputIndices.dim
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
        u[DBInputIndices.jerk] = self.jerk
        u[DBInputIndices.steering_angle_velocity] = self.steering_angle_velocity

        return u