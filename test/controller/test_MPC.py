import unittest

import numpy as np

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory

# instantiate double integrator model
vehicle_model = DoubleIntegrator(params=BMW3seriesParams(), dt=0.1)

# time
horizon = 20
delta_t = 0.1

# cost matrices
Q = np.eye(DIStateIndices.dim)
R = np.eye(DIInputIndices.dim)
P = np.eye(DIStateIndices.dim)

# Define the initial state
x0 = DIState(position_long=0.0, position_lat=2.0, velocity_long=5.0, velocity_lat=0.0)


time_state = [kk for kk in range(horizon+1)]
time_input = [kk for kk in range(horizon)]
sit_factory = DISITFactory()

# reference trajectory
x_ref_np = np.zeros((DIStateIndices.dim, horizon+1))
x_ref = sit_factory.trajectory_from_numpy_array(traj_np=x_ref_np,mode='state',time=time_state,t_0=0,delta_t=delta_t)
u_ref_np = np.zeros((DIInputIndices.dim, horizon))
u_ref = sit_factory.trajectory_from_numpy_array(traj_np=u_ref_np,mode='input',time=time_input,t_0=0,delta_t=delta_t)

# initial guess/ reference for linearization
x_init_np = np.zeros((DIStateIndices.dim, horizon+1))
x_init = sit_factory.trajectory_from_numpy_array(traj_np=x_init_np,mode='state',time=time_state,t_0=0,delta_t=delta_t)
u_init_np = np.zeros((DIInputIndices.dim, horizon))
u_init = sit_factory.trajectory_from_numpy_array(traj_np=u_init_np,mode='input',time=time_input,t_0=0,delta_t=delta_t)

# instantiate model predictive controller
solver = ModelPredictiveControl(vehicle_model, horizon, delta_t=delta_t, cost_xx=Q, cost_uu=R, cost_final=P)

# Solve the optimal control problem
x_sol, u_sol, iteration_history = solver._solve_ocp(x0, t_0 = 0.0, x_ref=x_ref, u_ref=u_ref, x_init=x_init, u_init=u_init)

# Print the solution
print("Optimal State Trajectory:\n", x_sol)
print("Optimal Control Inputs:\n", u_sol)
print("Iteration History (State, Control, Defect):\n", iteration_history)