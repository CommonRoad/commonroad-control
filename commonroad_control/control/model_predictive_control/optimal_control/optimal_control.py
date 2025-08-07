from abc import ABC, abstractmethod
import numpy as np
from typing import List, Tuple
from dataclasses import dataclass

from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


@dataclass(frozen=True)
class OCPSolverParameters(ABC):
    penalty_weight: float = 1000.0


class OptimalControl(ABC):
    def __init__(self,
                 vehicle_model: VehicleModelInterface,
                 sit_factory: StateInputTrajectoryFactoryInterface,
                 horizon: int,
                 delta_t: float,
                 ocp_parameters: OCPSolverParameters):

        self._vehicle_model = vehicle_model
        self._sit_factory = sit_factory
        self._ocp_parameters = ocp_parameters
        self._delta_t = delta_t

        # problem parameters
        self._horizon = horizon
        self._nx = self._vehicle_model.state_dimension
        self._nu = self._vehicle_model.input_dimension

    @abstractmethod
    def solve(self,
              x0: StateInterface,
              x_ref: TrajectoryInterface,
              u_ref: TrajectoryInterface,
              x_init: TrajectoryInterface,
              u_init: TrajectoryInterface) \
            -> Tuple[np.array, np.array, List[Tuple[np.array, np.array, np.array]]]:
        pass

