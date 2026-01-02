from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np

from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)


@dataclass(frozen=True)
class OCPSolverParameters(ABC):
    """
    Algorithm parameters for the optimal control problem solver.
    """

    penalty_weight: float = 1000.0


class OptimalControlSolverInterface(ABC):
    """
    Base cass for solving optimal control problems (OCPs). We consider OCPs of the form:

        minimize sum_{k=0}^horizon l_k(x(k),u(k)) + V_f(x(horizon))
        such that
                x(0) = x_init
                for k ={0,...,horizon-1}:   x(k+1) = f(x(k),u(k),0)
                                            u_lb <= u(k) <= u_ub
                for k={0,...,horizon}:      x_lb <= x(k) <= x_ub
                                            a_comb(k) <= 1

    where the stage cost function is
        l_k(x(k),u(k)) = (x(k) - x_ref(k))^T cost_xx (x(k) - x_ref(k)) + (u(k) - u_ref(k))^T cost_uu (u(k) - u_ref(k))
    and the terminal cost function is
        V_f(x(horizon)) = (x(horizon) - x_ref(horizon))^T cost_final (x(horizon) - x_ref(horizon))
    and a_comb(k) returns the normalized combined acceleration at each time step.
    """

    def __init__(
        self,
        vehicle_model: VehicleModelInterface,
        sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface,
        horizon: int,
        delta_t: float,
        ocp_parameters: OCPSolverParameters,
    ):
        """
        Initialize OCP solver
        :param vehicle_model: vehicle model for predicting future states - VehicleModelInterface
        :param sidt_factory: factory for creating States and Inputs from the optimal solution - StateInputDisturbanceTrajectoryFactoryInterface
        :param horizon: (discrete) prediction horizon/number of time steps - int
        :param delta_t: sampling time - float
        :param ocp_parameters: algorithm parameters
        """

        self.vehicle_model: VehicleModelInterface = vehicle_model
        self.sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface = sidt_factory
        self._ocp_parameters: OCPSolverParameters = ocp_parameters
        self.delta_t = delta_t

        # problem parameters
        self.horizon = horizon
        self._nx = self.vehicle_model.state_dimension
        self._nu = self.vehicle_model.input_dimension

    @abstractmethod
    def solve(
        self,
        x0: StateInterface,
        x_ref: TrajectoryInterface,
        u_ref: TrajectoryInterface,
        x_init: TrajectoryInterface,
        u_init: TrajectoryInterface,
    ) -> Tuple[
        TrajectoryInterface, TrajectoryInterface, List[Tuple[np.array, np.array]]
    ]:
        """
        Solves an instance of the optimal control problem given an initial state, a reference trajectory to be tacked and an initial guess for the state and control inputs.
        :param x0: initial state of the vehicle - StateInterface
        :param x_ref: state reference trajectory - TrajectoryInterface
        :param u_ref: input reference trajectory - TrajectoryInterface
        :param x_init: (optional) initial guess for the state trajectory -TrajectoryInterface
        :param u_init: (optional) initial state for the input trajectory - TrajectoryInterface
        :return: optimal state and input trajectory (TrajectoryInterface) and solution history
        """

        pass

    def reset_ocp_parameters(self, new_ocp_parameters: OCPSolverParameters):
        """
        Updates parameters of the algorithm for solving the OCP, e.g., if one wishes to change the number of iterations.
        :param new_ocp_parameters: updated algorithm parameters - OCPSolverParameters
        """
        self._ocp_parameters = new_ocp_parameters
