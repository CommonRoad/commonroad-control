import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


@dataclass
class KSTState(StateInterface):
    """
    State of kinematic single track. [x,y,v,a,psi,delta]
    """
    dim: int = 6
    x: float = None
    y: float = None
    v: float = None
    a: float = None
    psi: float = None
    delta: float = None

    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.ndarray:
        """
        converts class to numpy array
        :return: np.ndarray(dim,)
        """
        return np.asarray(
            [
                self.x,
                self.y,
                self.v,
                self.a,
                self.psi,
                self.delta
            ]
        )

    def set_values_from_np_array(self, np_array: np.ndarray) -> None:
        """
        :param np_array: (dim,) array of states
        """
        if np_array.size > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {np_array}")

        if np_array.shape[0] != self.dim:
            raise ValueError(f"input should be ({self.dim},) but is {np_array}")

        self.x = np_array[0]
        self.y = np_array[1]
        self.v = np_array[2]
        self.a = np_array[3]
        self.psi = np_array[4]
        self.delta = np_array[5]



