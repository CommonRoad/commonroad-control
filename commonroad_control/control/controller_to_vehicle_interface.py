from abc import ABC, abstractmethod
from typing import Union
import numpy as np

from commonroad_control.vehicle_dynamics.sidt_factory_interface import StateInputDisturbanceTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface


class Controller2VehicleInterface(ABC):
    def __init__(self, sit_factory: StateInputDisturbanceTrajectoryFactoryInterface):
        self._sit_factory = sit_factory

    @abstractmethod
    def input_from_controller(
            self,
            u_res_control: Union[np.array],
            x_res_control: Union[np.array, None] = None
    ) -> InputInterface:
        pass







