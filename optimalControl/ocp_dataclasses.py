from dataclasses import dataclass
import numpy as np


@dataclass
class TrajectoryPoint:
    dim: int = None

    def convert_to_array(self) -> np.array:
        pass

    def set_values_from_array(self, y_np: np.array):
        pass


@dataclass
class State(TrajectoryPoint):
    dim: int = 3
    position_x: float = None
    position_y: float = None
    orientation: float = None

    def convert_to_array(self) -> np.array:
        x_np = np.zeros((self.dim, 1), dtype=float)
        x_np[0] = self.position_x
        x_np[1] = self.position_y
        x_np[2] = self.orientation

        return x_np

    def set_values_from_array(self, x_np: np.array):
        self.position_x = x_np[0]
        self.position_y = x_np[1]
        self.orientation = x_np[2]


@dataclass
class ControlInput(TrajectoryPoint):
    dim: int = None

    def convert_to_array(self) -> np.array:
        pass

    def set_values_from_array(self, u_np: np.array):
        pass

