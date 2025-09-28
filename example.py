import numpy as np

from commonroad_control.simulation.simulation import Simulation
from commonroad_control.planning_converter.dummy_converter import DummyPlanningConverter
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from typing import List, Any


def main(simulate: bool = True) -> None:
    kst_sit_factory = KBSITFactory()
    params = BMW3seriesParams()
    traj_converter = DummyPlanningConverter(kb_factory=kst_sit_factory, vehicle_params=params)
    model = KinematicBicycle(params=params, delta_t=0.1)

    replannings = range(2)
    for replanning in replannings:
        ########## Dummy Trajectory from planner
        states = {0: KBState(position_x=5, position_y=0, velocity=5, heading=0, steering_angle=0),
                  1: KBState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
        planner_state_traj = KBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.1
        )

        inputs = {0: KBInput(acceleration=5, steering_angle_velocity=5), 1: KBInput(acceleration=3, steering_angle_velocity=3)}
        planner_input_traj = KBTrajectory(
            mode=TrajectoryMode.Input,
            points=inputs,
            t_0=0,
            delta_t=0.1
        )
        ###################################################


        simulator = Simulation(
            vehicle_model=model,
            state_input_factory=kst_sit_factory
        )

        # convert trajectory
        kst_state_traj: KBTrajectory = traj_converter.trajectory_p2c_kb(planner_state_traj, mode=TrajectoryMode.State)
        kst_input_traj: KBTrajectory = traj_converter.trajectory_p2c_kb(planner_input_traj, mode=TrajectoryMode.Input)

        print(f"-- Initial State --")
        print(kst_state_traj.initial_point)


        current_state: KBState = kst_state_traj.initial_point
        for t in range(2):
            # control dummy
            controller_state_output = current_state
            controller_u = kst_input_traj.points[t]

            if simulate:
                # simulate
                new_state, _, _ = simulator.simulate(
                    x0=controller_state_output, u=controller_u, time_horizon=0.1
                )

                # results
                print(f"--- Result ---")
                print(new_state)
                current_state = new_state
            else:
                # get real world output
                pass


        # input trajectory into planner
        traj_tr = traj_converter.trajectory_c2p_kb(kb_traj=kst_state_traj, mode=TrajectoryMode.State)




if __name__ == "__main__":
    main()


