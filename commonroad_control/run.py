import scipy
import numpy as np
import matplotlib.pyplot as plt

from models.constrained_integrator import (
    ImplementedModels,
    stage_cost,
    terminal_cost,
    dynamics,
    ineq_con_stage,
    ineq_con_terminal,
    get_params,
    StateConstrainedIntegrator,
)
from optimal_control.ocp_solver import SolverOptimalControl
from optimal_control.utils import Trajectory

# get problem parameters
params = get_params()

# desired final state: turn left maneuver
xf_turn_left = StateConstrainedIntegrator(
    position_x=16.0,
    position_y=2.0,
    orientation=0.1,
    velocity_x=20.0,
    velocity_y=0.5,
    velocity_orientation=0.0,
)

# initialize solver
solver = SolverOptimalControl(
    problem_params=params,
    dynamics=dynamics,
    stage_cost=stage_cost,
    terminal_cost=terminal_cost,
    ineq_con_stage=ineq_con_stage,
    ineq_con_terminal=ineq_con_terminal,
)


# compute initial guess
# -> states: linear interpolation between initial and desired final state xf
x0_np = params["x0"].convert_to_array()
x_init_interp_fun = scipy.interpolate.interp1d(
    [0.0, 1.0], np.hstack((x0_np, xf_turn_left.convert_to_array())), kind="linear"
)

x_init = Trajectory(
    N=params["N"], point_type="State", system=ImplementedModels.ConstrainedIntegrator
)

for kk in range(params["N"] + 1):
    x_init.set_point_from_array_at_time_step(x_init_interp_fun(kk / params["N"]), kk)
# control inputs: all set to zero (internally)

# solve optimal control problem
x_opt, u_opt = solver.solve(params["x0"], x_init, xf=xf_turn_left)
x_opt_np = np.hstack([x_opt.get_point_as_array_at_time_step(kk) for kk in range(params['N']+1)])

plt.plot(x_opt_np[0, :], x_opt_np[1, :], 'ko-')
plt.show()
