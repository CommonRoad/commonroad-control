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
    idx_position_x: int = 0
    idx_position_y: int = 1
    idx_orientation: int = 2

    def convert_to_array(self) -> np.array:
        x_np = np.zeros((self.dim, 1), dtype=float)
        x_np[self.idx_position_x] = self.position_x
        x_np[self.idx_position_y] = self.position_y
        x_np[self.idx_orientation] = self.orientation

        return x_np

    def set_values_from_array(self, x_np: np.array):
        self.position_x = x_np[self.idx_position_x]
        self.position_y = x_np[self.idx_position_y]
        self.orientation = x_np[self.idx_orientation]

    def project_position(self) -> np.array((2, 1)):
        return np.array([self.position_x, self.position_y])


@dataclass
class ControlInput(TrajectoryPoint):
    dim: int = None

    def convert_to_array(self) -> np.array:
        pass

    def set_values_from_array(self, u_np: np.array):
        pass
