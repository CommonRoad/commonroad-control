import numpy as np
from typing import List, Tuple
import scipy as sp
from pyparsing import version_info

from commonroad_control.control.controller import Controller
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control import OptimalControlSolver
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode


class ModelPredictiveControl(Controller):
    def __init__(self,
                 ocp_solver: OptimalControlSolver):

        # init base class
        super().__init__()

        # set optimal control problem solver
        self._ocp_solver = ocp_solver

        # store initial guess
        self._x_init = None
        self._u_init = None


    def compute_control_input(self, x0: StateInterface,
                              x_ref: TrajectoryInterface,
                              u_ref: TrajectoryInterface,
                              x_init: TrajectoryInterface = None,
                              u_init: TrajectoryInterface = None
                              ) -> InputInterface:


        # set initial guess for optimal control
        if self._x_init is None and self._u_init is None:
            x_init, u_init = self._initial_guess_linear_interpolation(
                x0=x0,
                xf=x_ref.final_state,
                t_0=x_ref.t_0)
        else:
            x_init, u_init = self._initial_guess_shift_solution()

        # solve optimal control problem
        x_opt, u_opt, _ = self._ocp_solver.solve(x0, x_ref, u_ref, x_init, u_init)

        # store solution as initial guess at next time step
        self._x_init = x_opt
        self._u_init = u_opt

        return u_opt.get_point_at_time_step(0)

    def _initial_guess_linear_interpolation(self,
                                            x0: StateInterface,
                                            xf: StateInterface,
                                            t_0: float) \
        -> Tuple[TrajectoryInterface, TrajectoryInterface]:
        """
        Computes an initial guess by linearly interpolating between the initial state x0 and the (desired) final
        state xf. The control inputs are set to zero.
        :param x0: initial state
        :param xf: (desired) final state
        :return: initial guess for the state and control inputs
        """

        # ... state: linear interpolation between x0 and xf
        x_init_interp_fun = sp.interpolate.interp1d(
            [0.0, 1.0],
            np.hstack((np.reshape(x0.convert_to_array(), (self._ocp_solver.vehicle_model.state_dimension.dim, 1)),
                       np.reshape(xf.convert_to_array().transpose(), (self._ocp_solver.vehicle_model.state_dimension.dim, 1)))),
            kind="linear"
        )
        x_init_np = np.zeros((self._ocp_solver.vehicle_model.state_dimension.dim, self._ocp_solver.horizon + 1))
        time_state = [kk for kk in range(self._ocp_solver.horizon+1)]
        for kk in range(self._ocp_solver.horizon + 1):
            x_init_np[:, kk] = x_init_interp_fun(kk / self._ocp_solver.horizon)
        x_init = self._ocp_solver.sit_factory.trajectory_from_numpy_array(
            traj_np=x_init_np,
            mode=TrajectoryMode.State,
            time=time_state,
            t_0=t_0,
            delta_t=self._ocp_solver.delta_t
        )
        # ... control inputs: set to zero
        u_init_np = np.zeros((self._ocp_solver.vehicle_model.input_dimension.dim, self._ocp_solver.horizon))
        time_input = [kk for kk in range(self._ocp_solver.horizon)]
        u_init = self._ocp_solver.sit_factory.trajectory_from_numpy_array(
            traj_np=u_init_np,
            mode=TrajectoryMode.Input,
            time=time_input,
            t_0=t_0,
            delta_t=self._ocp_solver.delta_t
        )

        return x_init, u_init

    def _initial_guess_shift_solution(self,
                                      ):
        # simulate using last control input from reference trajectory

        pass

    def clear_initial_guess(self):
        """
        Delete stored initial guess.
        :return:
        """
        self._x_init = None
        self._u_init = None
