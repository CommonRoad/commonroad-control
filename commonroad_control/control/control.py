from abc import ABC, abstractmethod
from typing import Any


class ControllerInterface(ABC):
    """
    Abstract controller interface
    """

    @abstractmethod
    def compute_control_input(
        self,
        *args,
    ) -> Any:
        pass
