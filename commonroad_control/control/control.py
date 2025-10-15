from abc import ABC, abstractmethod

from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface

from typing import Any


class ControllerInterface(ABC):

    @abstractmethod
    def compute_control_input(
        self,
        *args,
    ) -> Any:
        pass