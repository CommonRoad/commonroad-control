from abc import ABC, abstractmethod
from typing import Union
import numpy as np

from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface


class Controller2VehicleInterface(ABC):
    def __init__(self, sit_factory: StateInputTrajectoryFactoryInterface):
        self._sit_factory = sit_factory

    @abstractmethod
    def input_from_controller(
            self,
            u_res_control: Union[np.array],
            x_res_control: Union[np.array, None] = None
    ) -> InputInterface:
        pass







