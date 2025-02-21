import numpy as np
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


@dataclass
class KSTState(StateInterface):
    peter: int


    def __post_init__(self):
        super().__init__(dim=self.dim)

    def convert_to_array(self) -> np.ndarray:
        pass

    def set_values_from_np_array(self, np_array: np.ndarray):
        pass





if __name__ == "__main__":
    kst_state: KSTState = KSTState(dim=2, peter=4)
    print(kst_state)




