import numpy as np
from typing import Callable, Type, Dict, Optional, List
import casadi as cas

from commonroad_control.optimal_control.utils import rk4_integrator, Trajectory
from commonroad_control.optimal_control.ocp_dataclasses import State
from commonroad_control.optimal_control.collision_avoidance.constraint_factory import ConstraintFactory
from commonroad_control.optimal_control.collision_avoidance.ca_polyhedron import Polyhedron


class SolverOptimalControl:
    def __init__(
        self,
        problem_params: Dict,
        dynamics: Callable,
        stage_cost: Callable,
        terminal_cost: Callable,
        ineq_con_stage: Callable,
        ineq_con_terminal: Callable,
    ):

        # problem_params["n_ineq_con_ca"] = 0

        # extract problem data
        self._system = problem_params["system"]
        self._nx = problem_params["nx"]
        self._nu = problem_params["nu"]
        self._N = problem_params["N"]
        self._n_ineq_con_ca = problem_params["n_ineq_con_ca"]
        self._penaltyWeight = problem_params["penaltyWeight"]
        self._lbx = problem_params["lbx"].convert_to_array()
        self._ubx = problem_params["ubx"].convert_to_array()
        self._lbu = problem_params["lbu"].convert_to_array()
        self._ubu = problem_params["ubu"].convert_to_array()

        # cost and constraint functions
        # -> dummy variables for using CasADi
        x = cas.SX.sym("x", (problem_params["nx"], 1))
        u = cas.SX.sym("u", (problem_params["nu"], 1))
        xf = cas.SX.sym("x", (problem_params["nx"], 1))
        # -> setup cost + gradient
        self._stageCost = cas.Function("stageCost", [x, u], [stage_cost(x, u)])
        self._terminalCost = cas.Function(
            "terminalCost", [x, xf], [terminal_cost(x, xf)]
        )
        # -> discretize dynamical system
        self._dynamicsDT = cas.Function(
            "dynamicsDT", [x, u], [rk4_integrator(x, u, dynamics, problem_params["dt"])]
        )
        # -> setup constraints + gradient
        self._ineqConStage = cas.Function(
            "ineqConStage", [x, u], [cas.vertcat(*ineq_con_stage(x, u))]
        )
        self._ineqConTerminal = cas.Function(
            "ineqConTerminal", [x], [cas.vertcat(*ineq_con_terminal(x))]
        )

        # initialize constraint factory as well as related variables and parameters
        if self._n_ineq_con_ca == 0:
            self._ca_constraints = False
            self._max_iter = 1
        else:
            self._ca_constraints = True
            self._max_iter = 25
            self._constraint_factory = ConstraintFactory(N=self._N, nx=self._nx)
            self._varSlackCA = None
            self._ca_mat = None
            self._ca_rhs = None

        # parameters and variables of the optimal control problem
        self._varX = None
        self._varU = None
        self._x0 = None
        self._xf = None

        # setup solver
        self._solver = self.init_solver()

        # current optimal solution
        self._Xopt = Trajectory(self._N, "State", self._system)
        self._Uopt = []
        self._slackCAopt = []

    def init_solver(self) -> cas.Opti:
        """
        Modelling of the optimal control problem (using CasADi's 'opti stack'), i.e, declaration of variables and
        parameters (e.g. the initial state) as well as the cost and constraint functions.
        :return: parameterized optimal control problem
        """

        # optimal control problem
        opti = cas.Opti()

        # declare variables
        self._varX = opti.variable(self._nx, self._N + 1)
        self._varU = opti.variable(self._nu, self._N)
        if self._ca_constraints:
            self._varSlackCA = opti.variable(self._n_ineq_con_ca, self._N+1)

        # declare parameters
        # -> initial state
        self._x0 = opti.parameter(self._nx, 1)
        self._xf = opti.parameter(self._nx, 1)
        # -> collision avoidance constraints
        if self._ca_constraints:
            self._ca_mat = dict(zip(range(self._N+1),
                                    [opti.parameter(self._n_ineq_con_ca, self._nx) for kk in range(self._N+1)]))
            # self._ca_mat = dict.fromkeys(range(self._N+1), opti.parameter(self._n_ineq_con_ca, self._nx))
            self._ca_rhs = dict(zip(range(self._N+1),
                                    [opti.parameter(self._n_ineq_con_ca, 1) for kk in range(self._N+1)]))

        # ----------------- cost function -----------------
        cost = 0
        for kk in range(self._N):
            cost += self._stageCost(self._varX[:, kk], self._varU[:, kk])
            if self._ca_constraints:
                cost += self._penaltyWeight * cas.sum1(self._varSlackCA[:, kk])
        cost += self._terminalCost(self._varX[:, -1], self._xf)
        if self._ca_constraints:
            cost += self._penaltyWeight*cas.sum1(self._varSlackCA[:, self._N])
        opti.minimize(cost)

        # ----------------- constraints -----------------

        # initial state
        opti.subject_to(self._varX[:, 0] == self._x0)

        # stage constraints
        for kk in range(self._N):
            # -> dynamic feasibility
            opti.subject_to(
                self._varX[:, kk + 1]
                == self._dynamicsDT(self._varX[:, kk], self._varU[:, kk])
            )
            # -> collision avoidance constraints
            if self._ca_constraints:
                opti.subject_to(self._ca_mat[kk]@self._varX[:, kk] <= self._ca_rhs[kk] + self._varSlackCA[:, kk])
                opti.subject_to(self._varSlackCA[:, kk] >= 0)
            # -> state & input bounds
            opti.subject_to(
                [
                    self._lbx <= self._varX[:, kk],
                    self._varX[:, kk] <= self._ubx,
                    self._lbu <= self._varU[:, kk],
                    self._varU[:, kk] <= self._ubu,
                ]
            )
            # -> problem-specific inequality constraints
            opti.subject_to(
                self._ineqConStage(self._varX[:, kk], self._varU[:, kk]) <= 0
            )

        # terminal constraints
        # -> collision avoidance constraints
        if self._ca_constraints:
            opti.subject_to(self._ca_mat[self._N]@self._varX[:, self._N]
                            <= self._ca_rhs[self._N] + self._varSlackCA[:, self._N])
            opti.subject_to(self._varSlackCA[:, self._N] >= 0)
        # -> state bounds
        opti.subject_to(self._lbx <= self._varX[:, -1])
        opti.subject_to(self._varX[:, -1] <= self._ubx)
        # -> problem-specific constraints
        opti.subject_to(self._ineqConTerminal(self._varX[:, -1]) <= 0)

        # ----------------- finalize solver -----------------
        # set numerical backend
        opti.solver("ipopt")

        return opti

    def eval_penalized_objective(self) -> float:
        """
        Some iterative algorithms for solving non-convex optimal control/optimization problems measure progress towards
        the solution and determine the step size by evaluating a penalized objective function including the cost as well
        as all penalties incurred for violating constraints.
        :return: current cost value including penalties
        """

        pen_cost = 0

        # iterate over time steps
        for kk in range(self._N - 1):
            # -> stage cost
            pen_cost += self._stageCost(self._Xopt[:, kk], self._Uopt[:, kk]).full()
            # -> dynamic feasibility
            pen_cost += sum(
                abs(
                    self._Xopt[:, kk + 1]
                    - self._dynamicsDT(self._Xopt[:, kk], self._Uopt[:, kk]).full()
                )
            )
            # -> collision avoidance
            pen_cost += self._penaltyWeight * sum(self._slackCAopt[:, kk])

        # -> terminal cost
        pen_cost += self._penaltyWeight * self._terminalCost(self._Xopt[:, -1]).full()
        # -> collision avoidance
        pen_cost += sum(self._slackCAopt[:, -1])

        return pen_cost

    def solve(
        self,
        x0: Type[State],
        x_init: Optional[Trajectory] = None,
        u_init: Optional[Trajectory] = None,
        xf: Optional[State] = None,
        obstacles: Optional[Dict[int, List[Polyhedron]]] = None,

    ) -> (Trajectory, Trajectory):
        """
        Sets all parameters of the instance of the optimal control problem (OCP) to be solved, solves the OCP, and
        returns the optimal state and control input trajectories.
        :param x0:      initial state
        :param x_init:  initial guess for the state trajectory (initial state will be overridden by x0)
        :param u_init:  initial guess for the control inputs
        :param xf:      desired final state
        :param obstacles:
        :return: optimal state and control input trajectories
        """

        # set initial state
        self._solver.set_value(self._x0, x0.convert_to_array())

        # set desired final state (set to zero if no desired final state is given)
        if isinstance(xf, State):
            self._solver.set_value(self._xf, xf.convert_to_array())
        else:
            self._solver.set_value(self._xf, np.zeros((self._nx, 1), dtype=float))

        # initial guess (set to zero if none is provided)
        if not isinstance(x_init, Trajectory):
            x_init = Trajectory(N=self._N, point_type="State", system=self._system)
            x_init.set_trajectory_from_array(np.zeros((self._nx,self._N+1), dtype=float))
        if not isinstance(u_init, Trajectory):
            u_init = Trajectory(N=self._N, point_type="ControlInput", system=self._system)
            u_init.set_trajectory_from_array(np.zeros((self._nu,self._N), dtype=float))

        # update constraint factory
        if self._ca_constraints:
            self._constraint_factory.set_polyhedral_obstacles(obstacles)

        # iterate until convergence
        iterates = {0: {'x': np.hstack([x_init.get_point_as_array_at_time_step(kk) for kk in range(self._N+1)]),
                        'u': np.hstack([u_init.get_point_as_array_at_time_step(kk) for kk in range(self._N)])}}

        for ii in range(self._max_iter):
            # set initial values for the solver
            self._solver.set_initial(self._varX, iterates[0]['x'])
            self._solver.set_initial(self._varU, iterates[0]['u'])

            # set collision avoidance constraints
            if self._ca_constraints:
                for kk in range(self._N+1):
                    [tmp_mat, tmp_rhs] = self._constraint_factory.project_and_linearize_at_time_step(
                        kk, x_init.get_point_at_time_step(kk))
                    self._solver.set_value(self._ca_mat[kk], tmp_mat)
                    self._solver.set_value(self._ca_rhs[kk], tmp_rhs)

            # solve optimal control problem
            sol = self._solver.solve()

            # extract solution
            iterates[ii + 1] = {'x': sol.value(self._varX), 'u': sol.value(self._varU)}
            if self._ca_constraints:
                iterates[ii + 1]['slack'] = sol.value(self._varSlackCA)
            # check convergence
            if np.linalg.norm(iterates[ii+1]['x'].flatten() - iterates[ii]['x'].flatten()) <= 1e-6 \
                    and np.linalg.norm(iterates[ii+1]['u'].flatten() - iterates[ii]['u'].flatten()) <= 1e-6:
                break
            else:
                # set initial guess for the next iteration
                x_init.set_trajectory_from_array(iterates[ii+1]['x'])
                u_init.set_trajectory_from_array(iterates[ii+1]['u'])

        self._Xopt = iterates[ii+1]['x']
        self._Uopt = iterates[ii+1]['u']
        if self._ca_constraints:
            self._slackCAopt = iterates[ii+1]['slack']

        # convert solution back to State/ControlInput dataclasses
        x_opt = Trajectory(N=self._N, point_type="State", system=self._system)
        x_opt.set_trajectory_from_array(self._Xopt)
        u_opt = Trajectory(N=self._N, point_type="ControlInput", system=self._system)
        u_opt.set_trajectory_from_array(self._Uopt)

        return x_opt, u_opt
