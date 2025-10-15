import numpy as np
from typing import Tuple
import scipy as sp

from commonroad_control.control.control import ControllerInterface
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control import OptimalControlSolver
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode


class ModelPredictiveControl(ControllerInterface):
    def __init__(self,
                 ocp_solver: OptimalControlSolver):

        # init base class
        super().__init__()

        # set optimal control problem solver
        self.ocp_solver = ocp_solver

        # store initial guess
        self._x_init = None
        self._u_init = None


    def compute_control_input(self, x0: StateInterface,
                              x_ref: TrajectoryInterface,
                              u_ref: TrajectoryInterface,
                              x_init: TrajectoryInterface = None,
                              u_init: TrajectoryInterface = None
                              ) -> InputInterface:
        """
        Computes the control input by solving an optimal control problem.
        :param x0: initial state of the vehicle
        :param x_ref: state reference trajectory
        :param u_ref: input reference trajectory
        :param x_init: (optional) initial guess for the state trajectory
        :param u_init: (optional) initial state for the input trajectory
        :return: optimal control input
        """


        # set initial guess for optimal control: if no initial guess is provided (x_init = None and or u_init = None),
        # an initial guess is computed by shifting an old solution or linearly interpolating the state between the
        # initial state and the final reference state
        if x_init is None or u_init is None:
            if self._x_init is None and self._u_init is None:
                x_init, u_init = self._initial_guess_linear_interpolation(
                    x0=x0,
                    xf=x_ref.final_point,
                    t_0=x_ref.t_0)
            else:
                x_init, u_init = self._initial_guess_shift_solution(
                    u_ref)

        # solve optimal control problem
        x_opt, u_opt, _ = self.ocp_solver.solve(x0, x_ref, u_ref, x_init, u_init)

        # store solution as initial guess at next time step
        self._x_init = x_opt
        self._u_init = u_opt

        return u_opt.get_point_at_time_step(0)

    def _initial_guess_linear_interpolation(self,
                                            x0: StateInterface,
                                            xf: StateInterface,
                                            t_0: float
                                            ) -> Tuple[TrajectoryInterface, TrajectoryInterface]:
        """
        Computes an initial guess by linearly interpolating between the initial state x0 and the final reference state
        xf. The control inputs are set to zero.
        :param x0: initial state
        :param xf: final reference state
        :return: initial guess for the state and control inputs
        """

        # ... state: linear interpolation between x0 and xf
        x_init_interp_fun = sp.interpolate.interp1d(
            [0.0, 1.0],
            np.hstack((np.reshape(x0.convert_to_array(), (self.ocp_solver.vehicle_model.state_dimension, 1)),
                       np.reshape(xf.convert_to_array().transpose(), (self.ocp_solver.vehicle_model.state_dimension, 1)))),
            kind="linear"
        )
        x_init_np = np.zeros((self.ocp_solver.vehicle_model.state_dimension, self.ocp_solver.horizon + 1))
        time_state = [kk for kk in range(self.ocp_solver.horizon + 1)]
        for kk in range(self.ocp_solver.horizon + 1):
            x_init_np[:, kk] = x_init_interp_fun(kk / self.ocp_solver.horizon)
        x_init = self.ocp_solver.sit_factory.trajectory_from_numpy_array(
            traj_np=x_init_np,
            mode=TrajectoryMode.State,
            time=time_state,
            t_0=t_0,
            delta_t=self.ocp_solver.delta_t
        )
        # ... control inputs: set to zero
        u_init_np = np.zeros((self.ocp_solver.vehicle_model.input_dimension, self.ocp_solver.horizon))
        time_input = time_state
        time_input.pop()
        u_init = self.ocp_solver.sit_factory.trajectory_from_numpy_array(
            traj_np=u_init_np,
            mode=TrajectoryMode.Input,
            time=time_input,
            t_0=t_0,
            delta_t=self.ocp_solver.delta_t
        )

        return x_init, u_init

    def _initial_guess_shift_solution(self,
                                      u_ref: TrajectoryInterface
                                      ) -> Tuple[TrajectoryInterface, TrajectoryInterface]:
        """
        Computes an initial guess by shifting the solution from the previous time step and appending the last control
        input from the reference trajectory. The initial guess for the state at the end of the prediction horizon is
        obtained by simulating the system using the reference input.
        :param u_ref: reference trajectory - control input
        :return: initial guess for the state and control inputs
        """

        # extract states and input to be kept
        x_init_points = [self._x_init.get_point_at_time_step(kk) for kk in range(1, self.ocp_solver.horizon + 1)]
        u_init_points = [self._u_init.get_point_at_time_step(kk) for kk in range(1, self.ocp_solver.horizon)]

        # simulate vehicle model for num_steps using the reference control inputs
        # ... append last input from reference trajectory
        u_init_points.append(u_ref.final_point)
        # ... simulate
        x_init_points.append(
            self.ocp_solver.sit_factory.state_from_numpy_array(
                self.ocp_solver.vehicle_model.simulate_forward_dt(x_init_points[-1], u_init_points[-1]))
        )

        # convert to trajectory interface
        # ... state trajectory
        time_steps = [kk for kk in range(0, self.ocp_solver.horizon + 1)]
        x_init = self.ocp_solver.sit_factory.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(time_steps, x_init_points)),
            mode=TrajectoryMode.State,
            t_0=u_ref.t_0,
            delta_t=self.ocp_solver.delta_t
        )
        # ... input trajectory
        time_steps = [kk for kk in range(0, self.ocp_solver.horizon)]
        u_init = self.ocp_solver.sit_factory.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(time_steps, u_init_points)),
            mode=TrajectoryMode.Input,
            t_0=u_ref.t_0,
            delta_t=self.ocp_solver.delta_t
        )

        return x_init, u_init

    def clear_initial_guess(self):
        """
        Delete stored initial guess.
        :return:
        """
        self._x_init = None
        self._u_init = None

    @property
    def horizon(self):
        return self.ocp_solver.horizon

