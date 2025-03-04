import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.input_interface import InputInterface, InputInterfaceIndex


# TODO should be an enum so no initialization?
@dataclass(frozen=True)
class KSTInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs.
    """
    dim: int = 2
    acceleration: int = 0
    steering_angle_velocity: int = 1


# TODO: Move to python3.10 and use kw_only dataclass arg?
@dataclass()
class KSTInput(InputInterface):
    """
    Control input of the kinematic single track model.
    """
    dim: int = KSTInputIndices.dim
    acceleration: float = None
    steering_angle_velocity: float = None

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        u = np.zeros((self.dim,))
        u[KSTInputIndices.acceleration] = self.acceleration
        u[KSTInputIndices.steering_angle_velocity] = self.steering_angle_velocity

        return u
