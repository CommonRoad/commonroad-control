import cvxpy as cp
import numpy as np
from typing import List, Tuple

from commonroad_control.control.controller import Controller
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control import OptimalControl,OCPSolverParameters
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


class ModelPredictiveControl(Controller):
    def __init__(self, vehicle_model: VehicleModelInterface, horizon: int, delta_t: float,
                 cost_xx: np.array, cost_uu: np.array, cost_final: np.array,
                 soft_tr_penalty_weight=0.001, slack_penalty_weight=1000.0, max_iterations=10, tolerance=1e-3):

        # init base class
        super().__init__()

        # setup optimal control problem solver
        self._ocp_solver = ...

        self._vehicle_model = vehicle_model
        self._delta_t = delta_t


    def compute_control_input(self, x0: StateInterface, x_ref: TrajectoryInterface, u_ref: TrajectoryInterface,
                              x_init: TrajectoryInterface, u_init: TrajectoryInterface) -> InputInterface:
        _, u_opt, _ = self._ocp_solver.solve(x0, x_ref, u_ref, x_init, u_init)

        return u_opt.get_point_at_time_step(0)
