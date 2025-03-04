import cvxpy as cp
import numpy as np
from typing import List, Tuple

from commonroad_control.control.controller import Controller
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


class ModelPredictiveControl(Controller):
    def __init__(self, vehicle_model: VehicleModelInterface, horizon: int, delta_t: float,
                 cost_xx: np.array, cost_uu: np.array, cost_final: np.array,
                 penalty_weight=1e4, max_iterations=10, tolerance=1e-3):

        # init base class
        super().__init__()

        self._vehicle_model = vehicle_model
        self._delta_t = delta_t

        # optimal control problem parameters
        self._horizon = horizon
        self._nx = self._vehicle_model.state_dimension
        self._nu = self._vehicle_model.input_dimension
        self._cost_xx = cost_xx
        self._cost_uu = cost_uu
        self._cost_final = cost_final
        self._penalty_weight = penalty_weight
        self._max_iterations = max_iterations
        self._tolerance = tolerance

        # Set up decision variables
        self._x = cp.Variable((self._nx, self._horizon + 1))
        self._u = cp.Variable((self._nu, self._horizon))

        # Parameters
        self._par_x0 = cp.Parameter((self._nx,1))
        self._par_x_ref = cp.Parameter((self._nx, self._horizon + 1))
        self._par_u_ref = cp.Parameter((self._nu, self._horizon))
        self._par_x_lin = cp.Parameter((self._nx, self._horizon + 1))
        self._par_u_lin = cp.Parameter((self._nu, self._horizon))
        self._par_x_next = cp.Parameter((self._nx, self._horizon))
        self._par_A = cp.Parameter((self._nx, self._nx*self._horizon))
        self._par_B = cp.Parameter((self._nx, self._nu*self._horizon))

        # Create the problem
        self._problem = self._create_problem()

        # Initialize history
        self._iteration_history = []

    def _create_problem(self):
        # Initialize the cost function
        cost = 0

        # Constraints list
        constraints = [self._x[0] == self._par_x0]

        par_A = cp.reshape(self._par_A,(self._nx, self._nx, self._horizon))
        par_B = cp.reshape(self._par_B,(self._nx, self._nu, self._horizon))

        for kk in range(self._horizon):
            # cost function
            cost += (cp.quad_form(self._x[:, kk] - self._par_x_ref[:,kk], self._cost_xx)
                     + cp.quad_form(self._u[:, kk] - self._par_u_ref[:, kk], self._cost_uu))

            # dynamics constraint
            constraints += [self._x[:, kk + 1] == self._par_x_next[:, kk] \
                            + par_A[:, :, kk] @ (self._x[:, kk] - self._par_x_lin[:, kk]) \
                            + par_B[:, :, kk] @ (self._u[:, kk] - self._par_u_lin[:, kk])]
                            # + self._par_A[:, :, kk] @ (self._x[:, kk] - self._par_x_lin[:,kk]) \
                            # + self._par_B[:, :, kk] @ (self._u[:, kk] - self._par_u_lin[:, kk])]

            # Add state and control input bounds
            # x_lb, x_ub = self._vehicle_model.getStateBounds()
            u_lb, u_ub = self._vehicle_model.input_bounds()
            # constraints.append(self._x[:,kk] >= x_lb)
            # constraints.append(self._x[:, kk] <= x_ub)
            constraints.append(self._u[:, kk] >= u_lb.convert_to_array())
            constraints.append(self._u[:, kk] <= u_ub.convert_to_array())

            # # Add second-order cone constraints
            # soc_constraints = self._vehicle_model.nonlinConstraints(self._x[kk], self._u[kk], kk)
            # constraints += soc_constraints

        # Final state cost
        cost += cp.quad_form(self._x[:, self._horizon] - self._par_x_ref[:, self._horizon], self._cost_final)

        # (quadratic) soft trust-region penalty
        penalty = cp.norm(self._x - self._par_x_lin, 'fro') ** 2 + cp.norm(self._u - self._par_u_lin, 'fro') ** 2
        cost += self._penalty_weight * penalty

        # Define the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)

        return problem

    def compute_control_input(self, x0: StateInterface, x_ref: TrajectoryInterface, u_ref: TrajectoryInterface,
                              x_init: TrajectoryInterface, u_init: TrajectoryInterface) -> InputInterface:
        _, u_opt, _ = self._solve_ocp(x0, x_ref, u_ref, x_init, u_init)

        return u_opt.get_point_at_time_step(0)

    def _solve_ocp(self,
                   x0: StateInterface,
                   t_0: float,
                   x_ref: TrajectoryInterface,
                   u_ref: TrajectoryInterface,
                   x_init: TrajectoryInterface,
                   u_init: TrajectoryInterface) \
            -> Tuple[np.array, np.array, List[Tuple[np.array, np.array, np.array]]]:

        # reset iteration history
        self._iteration_history = []

        # set initial state
        self._par_x0.value = np.reshape(x0.convert_to_array(), (self._nx, 1))

        time_state = [t_0 + self._delta_t*kk for kk in range(self._horizon+1)]
        time_input = [t_0 + self._delta_t*kk for kk in range(self._horizon)]

        # set reference trajectories (cost function)
        self._par_x_ref.value = x_ref.convert_to_numpy_array(time_state)
        self._par_u_ref.value = u_ref.convert_to_numpy_array(time_input)

        # set reference trajectories for linearizing the vehicle model
        self._par_x_lin.value = x_init.convert_to_numpy_array(time_state)
        self._par_u_lin.value = u_init.convert_to_numpy_array(time_input)

        x_sol = []
        u_sol = []

        for _ in range(self._max_iterations):
            # linearize the nonlinear vehicle model
            x_next = []
            A = []
            B = []
            for kk in range(self._horizon):
                tmp_x, tmp_A, tmp_B = self._vehicle_model.linearize_dt_at(
                    self._par_x_lin.value[:, kk], self._par_u_lin.value[:, kk])
                x_next.append(tmp_x)
                A.append(tmp_A)
                B.append(tmp_B)
            self._par_x_next.value = np.hstack(x_next)
            self._par_A.value = np.hstack(A)
            self._par_B.value = np.hstack(B)

            # solve the optimal control problem
            self._problem.solve()

            # get the solution
            x_sol = self._x.value
            u_sol = self._u.value

            # compute the defect
            defect = np.linalg.norm(self._par_x_lin.value - x_sol, 'fro')

            # save solution
            self._iteration_history.append((x_sol.copy(), u_sol.copy(), defect))

            # check convergence
            if (np.linalg.norm(x_sol - self._par_x_lin.value) < self._tolerance
                    and np.linalg.norm(u_sol - self._par_u_lin.value) < self._tolerance):
                break

            # Update the initial guess
            self._par_x_lin.value = x_sol
            self._par_u_lin.value = u_sol

        return x_sol, u_sol, self._iteration_history
