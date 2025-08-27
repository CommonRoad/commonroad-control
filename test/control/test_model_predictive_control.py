import unittest

import numpy as np
import cvxpy as cp

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory
from commonroad_control.control.model_predictive_control.mpc_controller2vehicle import MPCController2Vehicle


class TestModelPredictivecontrol(unittest.TestCase):
    """
    Tests model predictive control.
    """

    def test_di_control2origin(self) -> None:
        """
        Simple test case: steer the double-integrator system to the orign..
        :return:
        """

        # time
        horizon_mpc = 10
        horizon_sim = 10
        delta_t = 0.1

        # instantiate double integrator model
        vehicle_model = DoubleIntegrator(
            params=BMW3seriesParams(),
            dt=delta_t)

        # cost matrices
        Q = np.eye(DIStateIndices.dim)
        R = np.zeros((DIInputIndices.dim, DIInputIndices.dim))
        P = np.eye(DIStateIndices.dim)

        # Define the initial state
        # initial state for 1-step - can be steered into the origin for delta t = 0.1
        u = -1.2
        pos = (-0.5*delta_t**2 + delta_t**2)*u
        v = (-delta_t*horizon)*u
        x0 = DIState(position_long=pos, velocity_long=v, position_lat=pos, velocity_lat=v)

        time_state = [kk for kk in range(horizon+1)]
        time_input = [kk for kk in range(horizon)]
        sit_factory = DISITFactory()

        # reference trajectory
        x_ref_np = np.zeros((DIStateIndices.dim, horizon+1))
        # x_ref_np[DIStateIndices.position_long,:] = [x0.velocity_long*kk*delta_t for kk in range(horizon+1)]
        x_ref = sit_factory.trajectory_from_numpy_array(traj_np=x_ref_np, mode='state', time=time_state,  t_0=0, delta_t=delta_t)
        u_ref_np = np.zeros((DIInputIndices.dim, horizon))
        u_ref = sit_factory.trajectory_from_numpy_array(traj_np=u_ref_np, mode='input', time=time_input,t_0=0, delta_t=delta_t)

        # initial guess/ reference for linearization
        x_init_np = np.zeros((DIStateIndices.dim, horizon+1))
        x_init = sit_factory.trajectory_from_numpy_array(traj_np=x_init_np, mode='state', time=time_state, t_0=0, delta_t=delta_t)
        u_init_np = np.zeros((DIInputIndices.dim, horizon))
        u_init = sit_factory.trajectory_from_numpy_array(traj_np=u_init_np, mode='input', time=time_input, t_0=0, delta_t=delta_t)

        # instantiate model predictive control
        solver = ModelPredictiveControl(vehicle_model, horizon, delta_t=delta_t, cost_xx=Q, cost_uu=R, cost_final=P)

        # Solve the optimal control problem
        x_sol, u_sol, iteration_history = solver._solve_ocp(x0, t_0=0.0, x_ref=x_ref, u_ref=u_ref, x_init=x_init, u_init=u_init)

        var_x = cp.Variable((solver._nx, horizon+1))
        var_u = cp.Variable((solver._nu, horizon))

        par_x0 = cp.Parameter((solver._nx,))

        cost = 0

        _, A, B = vehicle_model.linearize_dt_at(np.zeros((4, 1)), np.zeros((2, 1)))

        # Constraints list
        constraints = [solver._x[:, 0] == par_x0]

        for kk in range(horizon):
            cost += cp.sum_squares(var_x[:, kk + 1]) + cp.sum_squares(var_u[:, kk])
            constraints += [var_x[:, kk + 1] == A @ var_x[:, kk] + B @ var_u[:, kk],
                            cp.norm(var_u[:, kk], "inf") <= 1]

        # sums problem objectives and concatenates constraints.
        problem = cp.Problem(cp.Minimize(cost), constraints)

        par_x0.value = x0.convert_to_array()

        problem.solve(solver='CLARABEL', verbose=True)

        # Print the solution
        print("Optimal State Trajectory:\n", x_sol)
        print("Optimal Control Inputs:\n", u_sol)
        print("Iteration History (State, Control, Defect):\n", iteration_history)

