import logging
from dataclasses import dataclass
from typing import List, Tuple

import cvxpy as cp
import numpy as np

from commonroad_control.control.model_predictive_control.optimal_control.optimal_control import (
    OCPSolverParameters,
    OptimalControlSolver,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SCvxParameters(OCPSolverParameters):
    """
    Parameters for the Successive Convexification algorithm.
    """

    penalty_weight: float = (
        1000.0  # weight for the (exact) penalty of non-zero values of slack variables
    )
    max_iterations: int = 10  # maximum number of convex programming iterations
    soft_tr_penalty_weight: float = 0.001  # penalty for soft trust-region terms
    convergence_tolerance: float = 1e-3  # tolerance for convergence
    feasibility_tolerance: float = 1e-4  # tolerance for dynamical feasibility

    def __post_init__(self):
        super().__init__(penalty_weight=self.penalty_weight)


class OptimalControlSCvx(OptimalControlSolver):
    """
    Successive convexification algorithm for solving non-convex optimal control problems (OCP) based on
        [1] T. P. Reynolds et al. "A Real-Time Algorithm for Non-Convex Powered Descent Guidance", AIAA Scitech Forum, 2020

    We consider OCPs of the form:

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

    The algorithm successively convexifies this problem around the solution from the previous time step and solves these
    convex programming approximations until convergence.
    """

    def __init__(
        self,
        vehicle_model: VehicleModelInterface,
        sit_factory: StateInputDisturbanceTrajectoryFactoryInterface,
        horizon: int,
        delta_t: float,
        cost_xx: np.ndarray,
        cost_uu: np.ndarray,
        cost_final: np.ndarray,
        ocp_parameters: SCvxParameters = SCvxParameters,
    ):
        """
        Initialize OCP solver
        :param vehicle_model: vehicle model for predicting future states - VehicleModelInterface
        :param sit_factory: factory for creating States and Inputs from the optimal solution - StateInputDisturbanceTrajectoryFactoryInterface
        :param horizon: (discrete) prediction horizon/number of time steps - int
        :param delta_t: sampling time - float
        :param cost_xx: stage cost weighting matrix for the states, symmetric positive-semidefinite matrix - np.array
        :param cost_uu: stage cost weighting matrix for the inputs, symmetric positive-semidefinite matrix - np.array
        :param cost_final: terminal cost weighting matrix, symmetric positive-semidefinite matrix - np.array
        :param ocp_parameters: algorithm parameters
        """

        # init base class
        super().__init__(
            vehicle_model=vehicle_model,
            sit_factory=sit_factory,
            ocp_parameters=ocp_parameters,
            horizon=horizon,
            delta_t=delta_t,
        )

        # optimal control problem parameters
        self._horizon = horizon
        self._nx = self.vehicle_model.state_dimension
        self._nu = self.vehicle_model.input_dimension
        self._cost_xx = cost_xx
        self._cost_uu = cost_uu
        self._cost_final = cost_final

        # optimization variables
        # ... predicted states and control inputs
        self._x = cp.Variable((self._nx, self._horizon + 1))
        self._u = cp.Variable((self._nu, self._horizon))
        # ... virtual control inputs (see [1, Eq. (14)])
        self._virtual_control_pos = cp.Variable((self._nx, self._horizon))
        self._virtual_control_neg = cp.Variable((self._nx, self._horizon))

        # parameters of the optimal control problem
        # ... initial state
        self._par_x0 = cp.Parameter((self._nx,))
        # ... reference trajectories
        self._par_x_ref = cp.Parameter((self._nx, self._horizon + 1))
        self._par_u_ref = cp.Parameter((self._nu, self._horizon))
        # ... reference point for linearization of the vehicle dynamics
        self._par_x_lin = cp.Parameter((self._nx, self._horizon + 1))
        self._par_u_lin = cp.Parameter((self._nu, self._horizon))
        # ... linearization of vehicle dynamics (see [1, Eq. (14)])
        self._par_x_next = cp.Parameter((self._nx, self._horizon))
        self._par_A = cp.Parameter((self._nx, self._nx * self._horizon))
        self._par_B = cp.Parameter((self._nx, self._nu * self._horizon))
        # ... convexification of the (normalized) combined acceleration
        self._par_a_long = cp.Parameter(self._horizon, "a_long")
        self._par_a_lat = cp.Parameter(self._horizon, "a_lat")
        self._par_jac_a_long_x = cp.Parameter((self._horizon, self._nx), "jac_a_long_x")
        self._par_jac_a_long_u = cp.Parameter((self._horizon, self._nu), "jac_a_long_u")
        self._par_jac_a_lat_x = cp.Parameter((self._horizon, self._nx), "jac_a_lat_x")
        self._par_jac_a_lat_u = cp.Parameter((self._horizon, self._nu), "jac_a_lat_u")

        # Initialize history
        self._iteration_history = []

        # setup solver
        self._ocp = self._setup_solver()

    def _setup_solver(self):
        """
        Creates a parameterized convex programming approximation of the OCP using CVXPY.
        """

        # initialize cost function
        cost = 0.0

        # initial state constraint
        constraints = [self._x[:, 0] == self._par_x0]

        for kk in range(self._horizon):
            # cost function
            cost += cp.quad_form(
                self._x[:, kk] - self._par_x_ref[:, kk], self._cost_xx
            ) + cp.quad_form(self._u[:, kk] - self._par_u_ref[:, kk], self._cost_uu)

            # dynamics constraint
            constraints += [
                self._x[:, kk + 1]
                == self._par_x_next[:, kk]
                + self._virtual_control_pos[:, kk]
                - self._virtual_control_neg[:, kk]
                + (
                    self._par_A[:, [kk * self._nx + ii for ii in range(self._nx)]]
                    @ (self._x[:, kk] - self._par_x_lin[:, kk])
                )
                + (
                    self._par_B[:, [kk * self._nu + ii for ii in range(self._nu)]]
                    @ (self._u[:, kk] - self._par_u_lin[:, kk])
                )
            ]

            # acceleration constraints
            constraints += [
                (
                    self._par_a_long[kk]
                    + self._par_jac_a_long_x[kk, :]
                    @ (self._x[:, kk] - self._par_x_lin[:, kk])
                    + self._par_jac_a_long_u[kk, :]
                    @ (self._u[:, kk] - self._par_u_lin[:, kk])
                )
                ** 2
                + (
                    self._par_a_lat[kk]
                    + self._par_jac_a_lat_x[kk, :]
                    @ (self._x[:, kk] - self._par_x_lin[:, kk])
                    + self._par_jac_a_lat_u[kk, :]
                    @ (self._u[:, kk] - self._par_u_lin[:, kk])
                )
                ** 2
                <= 1
            ]

        # terminal cost
        cost += cp.quad_form(
            self._x[:, self._horizon] - self._par_x_ref[:, self._horizon],
            self._cost_final,
        )

        # control input bounds
        u_lb, u_ub = self.vehicle_model.input_bounds()
        constraints.append(
            self._u
            >= np.tile(
                np.reshape(u_lb.convert_to_array(), (self._nu, 1)), (1, self._horizon)
            )
        )
        constraints.append(
            self._u
            <= np.tile(
                np.reshape(u_ub.convert_to_array(), (self._nu, 1)), (1, self._horizon)
            )
        )

        # # state bounds
        # x_mat_lb, x_lb, x_mat_ub, x_ub = self.vehicle_model.state_bounds()
        # constraints += [x_lb <= x_mat_lb @ self._x,
        #                 x_mat_ub @ self._x <= x_ub]

        # bounds on virtual control inputs
        constraints += [self._virtual_control_pos >= 0, self._virtual_control_neg >= 0]

        # penalize virtual control inputs
        cost += self.ocp_parameters.penalty_weight * (
            cp.sum(self._virtual_control_neg) + cp.sum(self._virtual_control_pos)
        )

        # (quadratic) soft trust-region penalty
        cost += self.ocp_parameters.soft_tr_penalty_weight * (
            cp.sum_squares(cp.vec(self._x - self._par_x_lin, order="F"))
            + cp.sum_squares(cp.vec(self._u - self._par_u_lin, order="F"))
        )

        # Define the optimization problem
        problem = cp.Problem(cp.Minimize(cost), constraints)

        return problem

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
        Solves an instance of the optimal control problem given an initial state, a reference trajectory to be tracked
        and an initial guess for the state and control inputs.
        :param x0: initial state of the vehicle - StateInterface
        :param x_ref: state reference trajectory - TrajectoryInterface
        :param u_ref: input reference trajectory - TrajectoryInterface
        :param x_init: (optional) initial guess for the state trajectory -TrajectoryInterface
        :param u_init: (optional) initial state for the input trajectory - TrajectoryInterface
        :return: optimal state and input trajectory (TrajectoryInterface) and solution history
        """

        # reset iteration history
        self._iteration_history = []

        # set initial state
        self._par_x0.value = x0.convert_to_array()

        t_0 = x_ref.t_0
        time_state = [t_0 + self.delta_t * kk for kk in range(self._horizon + 1)]
        time_input = [t_0 + self.delta_t * kk for kk in range(self._horizon)]

        # set reference trajectories (cost function)
        self._par_x_ref.value = x_ref.convert_to_numpy_array(time_state)
        self._par_u_ref.value = u_ref.convert_to_numpy_array(time_input)

        # set reference trajectories for linearizing the vehicle model
        self._par_x_lin.value = x_init.convert_to_numpy_array(time_state)
        self._par_u_lin.value = u_init.convert_to_numpy_array(time_input)

        x_sol = []
        u_sol = []

        for num_iter in range(self.ocp_parameters.max_iterations):
            # linearize the nonlinear vehicle dynamics
            x_next = []
            A = []
            B = []
            for kk in range(self._horizon):
                tmp_x, tmp_A, tmp_B = self.vehicle_model.linearize_dt_nom_at(
                    self._par_x_lin.value[:, kk], self._par_u_lin.value[:, kk]
                )
                x_next.append(tmp_x)
                A.append(tmp_A)
                B.append(tmp_B)
            self._par_x_next.value = np.hstack(x_next)
            self._par_A.value = np.hstack(A)
            self._par_B.value = np.hstack(B)

            # linearize acceleration constraints
            a_long = []
            a_lat = []
            jac_a_long_x = []
            jac_a_long_u = []
            jac_a_lat_x = []
            jac_a_lat_u = []
            for kk in range(self._horizon):
                (
                    tmp_a_long,
                    tmp_a_lat,
                    tmp_jac_a_long_x,
                    tmp_jac_a_long_u,
                    tmp_jac_a_lat_x,
                    tmp_jac_a_lat_u,
                ) = self.vehicle_model.linearize_acceleration_constraints_at(
                    self._par_x_lin.value[:, kk], self._par_u_lin.value[:, kk]
                )
                a_long.append(tmp_a_long)
                a_lat.append(tmp_a_lat)
                jac_a_long_x.append(tmp_jac_a_long_x)
                jac_a_long_u.append(tmp_jac_a_long_u)
                jac_a_lat_x.append(tmp_jac_a_lat_x)
                jac_a_lat_u.append(tmp_jac_a_lat_u)
            self._par_a_long.value = np.vstack(a_long).squeeze()
            self._par_a_lat.value = np.vstack(a_lat).squeeze()
            self._par_jac_a_long_x.value = np.vstack(jac_a_long_x)
            self._par_jac_a_long_u.value = np.vstack(jac_a_long_u)
            self._par_jac_a_lat_x.value = np.vstack(jac_a_lat_x)
            self._par_jac_a_lat_u.value = np.vstack(jac_a_lat_u)

            # solve the optimal control problem
            self._ocp.solve(solver="CLARABEL", verbose=False)
            logger.debug(
                "Solver: %s | Iteration: %s | Status: %s | Optimal value: %s",
                self._ocp.solver_stats.solver_name,
                num_iter + 1,
                self._ocp.solution.status,
                self._ocp.solution.opt_val,
            )

            # check feasibility
            if "infeasible" in self._ocp.status or "unbounded" in self._ocp.status:
                logger.error(
                    f"SCvx convex programming solver failed. Status: {self._ocp.status}"
                )
                raise ValueError(
                    f"SCvx convex programming solver failed. Status: {self._ocp.status}"
                )

            # extract (candidate) solution
            x_sol = self._x.value
            u_sol = self._u.value

            # save solution
            self._iteration_history.append((x_sol.copy(), u_sol.copy()))

            # check convergence (see [1, Sec. II-E])
            conv_x = float(np.max(np.abs(x_sol - self._par_x_lin.value)))
            conv_u = float(np.max(np.abs(u_sol - self._par_u_lin.value)))
            if (
                conv_x < self.ocp_parameters.convergence_tolerance
                and conv_u < self.ocp_parameters.convergence_tolerance
            ):
                break

            # Update the initial guess
            self._par_x_lin.value = x_sol
            self._par_u_lin.value = u_sol

        # check feasibility
        # ... compute the defect (see [1, Alg. 2, l. 12-15] - for efficiency, we only compute the defect once the algorithm terminated)
        defect = self._compute_defect(x_sol, u_sol)
        if (float(np.max(defect)) > self.ocp_parameters.feasibility_tolerance) and (
            self.ocp_parameters.max_iterations > 1
        ):
            logger.debug(
                "SCvx algorithm converged to a dynamically infeasible solution!"
            )

        x_sol = self.sit_factory.trajectory_from_numpy_array(
            traj_np=x_sol,
            mode=TrajectoryMode.State,
            time_steps=[kk for kk in range(self._horizon + 1)],
            t_0=t_0,
            delta_t=self.delta_t,
        )

        u_sol = self.sit_factory.trajectory_from_numpy_array(
            traj_np=u_sol,
            mode=TrajectoryMode.Input,
            time_steps=[kk for kk in range(self._horizon)],
            t_0=t_0,
            delta_t=self.delta_t,
        )

        return x_sol, u_sol, self._iteration_history

    def _compute_defect(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """
        Computes the defect, which is the error in the predicted state trajectory, at each time step.
        :param x: predicted state trajectory - np.ndarray
        :param u: candidate control input trajectory - np.ndarray
        :return: array storing the defect at each time step - np.ndarray
        """
        err = x[:, 1 : self._horizon + 1] - np.column_stack(
            [
                self.vehicle_model.simulate_dt_nom(x[:, kk], u[:, kk])
                for kk in range(self._horizon)
            ]
        )
        return np.linalg.norm(err, axis=0)

    @property
    def ocp_parameters(self) -> SCvxParameters:
        return self._ocp_parameters
