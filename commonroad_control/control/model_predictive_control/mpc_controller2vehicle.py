from typing import Union

import numpy as np

from commonroad_control.control.controller_to_vehicle_interface import (
    Controller2VehicleInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)


class MPCController2Vehicle(Controller2VehicleInterface):
    def __init__(self, sit_factory: StateInputDisturbanceTrajectoryFactoryInterface):

        # init base class
        super().__init__(sit_factory=sit_factory)

    def input_from_controller(
        self,
        u_res_control: Union[np.array],
        x_res_control: Union[np.array, None] = None,
    ) -> InputInterface:

        # input args check

        # only the first element from the input trajectory is applied
        u_out_np = u_res_control[:, 0]
        return self._sit_factory.input_from_numpy_array(u_out_np)
