import matplotlib.pyplot as plt
import scipy
import numpy as np

from models.constrained_integrator import (
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
from commonroad_control.models.constrained_integrator import ImplementedModels
from commonroad_control.optimal_control.collision_avoidance.obstacles_Elchtest import get_obstacles

# get problem parameters
params = get_params()
params['N'] = 35
params['x0'].velocity_x = 13.9

# compute obstacles - width of vehicle set to zero since we model the vehicle as a point mass
obstacles = get_obstacles(w=0, N=params['N'])
params['n_ineq_con_ca'] = len(obstacles[0])

# desired final state: exit section 5 of maneuver (see get_obstacles)
xf_elchtest = StateConstrainedIntegrator(
    position_x=70.0,
    position_y=0.0,
    orientation=0.0,
    velocity_x=params['x0'].velocity_x,
    velocity_y=0.0,
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
    [0.0, 1.0], np.hstack((x0_np, xf_elchtest.convert_to_array())), kind="linear"
)

x_init = Trajectory(
    N=params["N"], point_type="State", system=ImplementedModels.ConstrainedIntegrator
)

for kk in range(params["N"] + 1):
    x_init.set_point_from_array_at_time_step(x_init_interp_fun(kk / params["N"]), kk)
# control inputs: all set to zero (internally)

# solve optimal control problem
x_opt, u_opt = solver.solve(params["x0"], x_init, xf=xf_elchtest, obstacles=obstacles)

x_opt_np = np.hstack([x_opt.get_point_as_array_at_time_step(kk) for kk in range(params['N']+1)])

# plot solution
for p in obstacles[0]:
    p.plot()
plt.plot(x_opt_np[0, :], x_opt_np[1, :], 'ko-')
plt.show()

