from abc import ABC, abstractmethod
import numpy as np
from typing import Any, Union, Dict



class StateInputTrajectoryFactoryInterface(ABC):


    @abstractmethod
    def state_from_numpy_array(
            self,
            arr: np.ndarray,
    ) -> Union[Any]:
        pass

    @abstractmethod
    def input_from_numpy_array(
            self,
            arr: np.ndarray
    ) -> Union[Any]:
        pass
