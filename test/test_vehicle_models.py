from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.simulation.simulation import Simulation


# init vehicle model
kst = KinematicSingleStrack(params=BMW3seriesParams, dt=0.1)

# init simulation
sim = Simulation(kst)

# set initial state and control input
x0 = KSTState(position_x=0.0, position_y=0.0, velocity=10.0, acceleration=0.0, heading=0.0, steering_angle=0.1)
u = KSTInput(jerk=0.0, steering_angle_velocity=0.0)

# simulate
x_sim = sim.simulate(x0, u, time_horizon=1.0)

