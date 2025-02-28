import numpy as np

from commonroad_control.simulation.simulation import Simulation
from commonroad_control.planning_converter.dummy_converter import DummyPlanningConverter
from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from typing import List, Any


def main(simulate: bool = True) -> None:
    kst_sit_factory = KSTSITFactory()
    params = BMW3seriesParams()
    traj_converter = DummyPlanningConverter(kst_factory=kst_sit_factory, vehicle_params=params)
    model = KinematicSingleStrack(params=params, dt=0.1)

    replannings = range(2)
    for replanning in replannings:
        ########## Dummy Trajectory from planner
        states = {0: KSTState(position_x=5, position_y=0, velocity=5, acceleration=0, heading=0, steering_angle=0),
                  1: KSTState(position_x=3, position_y=3, velocity=5, acceleration=3, heading=3, steering_angle=3)}
        planner_state_traj = KSTTrajectory(
            mode='state',
            states=states,
            t_0=0,
            delta_t=0.1
        )

        inputs = {0: KSTInput(jerk=5, steering_angle_velocity=5), 1: KSTInput(jerk=3, steering_angle_velocity=3)}
        planner_input_traj = KSTTrajectory(
            mode='input',
            states=inputs,
            t_0=0,
            delta_t=0.1
        )
        ###################################################


        simulator = Simulation(vehicle_model=model)

        # convert trajectory
        kst_state_traj: KSTTrajectory = traj_converter.trajectory_p2c_kst(planner_state_traj, mode='state')
        kst_input_traj: KSTTrajectory = traj_converter.trajectory_p2c_kst(planner_input_traj, mode='input')

        print(f"-- Initial State --")
        print(kst_state_traj.initial_state)


        current_state: KSTState = kst_state_traj.initial_state
        for t in range(2):
            # controller dummy
            controller_state_output = current_state
            controller_u = kst_input_traj.states[t]

            if simulate:
                # simulate
                new_state_np: np.ndarray = simulator.simulate(
                    x0=controller_state_output, u=controller_u, time_horizon=0.1
                )

                new_state: KSTState = kst_sit_factory.state_from_numpy_array(new_state_np)
                # results
                print(f"--- Result ---")
                print(new_state)
                current_state = new_state
            else:
                # get real world output
                pass


        # input trajectory into planner
        traj_tr = traj_converter.trajectory_c2p_kst(kst_traj=kst_state_traj, mode='state')




if __name__ == "__main__":
    main()


