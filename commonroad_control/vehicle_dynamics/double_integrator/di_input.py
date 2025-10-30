import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.input_interface import InputInterface, InputInterfaceIndex


@dataclass(frozen=True)
class DIInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs.
    """
    dim: int = 2
    acceleration_long: int = 0
    acceleration_lat: int = 1


@dataclass
class DIInput(InputInterface):
    """
    Control input of the double integrator model.
    """
    acceleration_long: float = None
    acceleration_lat: float = None

    @property
    def dim(self):
        return DIInputIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        u_np = np.zeros((self.dim,))
        u_np[DIInputIndices.acceleration_long] = self.acceleration_long
        u_np[DIInputIndices.acceleration_lat] = self.acceleration_lat

        return u_np

