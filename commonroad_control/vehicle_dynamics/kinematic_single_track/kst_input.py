import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.input_interface import InputInterface


@dataclass
class KSTInput(InputInterface):
    """
    Input of kinematic single track. The model input is Jerk (j) and steering angle rate (delta dot)
    """
    j: float
    delta_dot: float


    def __post_init__(self):
        super().__init__(dim=2)

    def convert_to_array(self) -> np.ndarray:
        """
        converts class to numpy array
        :return: np.ndarray(dim,)
        """
        return np.asarray(
            [
                self.j,
                self.delta_dot
            ]
        )

    def set_values_from_np_array(self, np_array: np.ndarray) -> None:
        """
        :param np_array: (dim,) array of states
        """
        if np_array.size() > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {np_array}")

        if np_array.shape[0] != self.dim:
            raise ValueError(f"input should be ({self.dim},) but is {np_array}")

        self.j = np_array[0]
        self.delta_dot = np_array[1]
