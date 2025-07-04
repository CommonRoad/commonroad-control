import copy
import random
import unittest
import ast
from pathlib import Path


import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_rp.state import ReactivePlannerState
from commonroad.scenario.state import InputState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.simulation.simulation import Simulation
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from rp_test import main as rpmain

from typing import List, Tuple

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories, make_gif

from commonroad_control.control.pid.pid_controller import PIDController




def main() -> None:
    scenario_file = Path(__file__).parents[0] / "scenarios" / "ZAM_Tjunction-1_42_T-1.xml"

    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    controller_time: float = 0.01
    planner_time: float = 0.1


    state_input_factory = KSTSITFactory()

    kst_traj, kst_input = execute_planner()
    vehicle_params: VehicleParameters = BMW3seriesParams()
    vehicle_model: KinematicSingleStrack = KinematicSingleStrack(params=vehicle_params, dt=controller_time)
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model,
        state_input_factory=state_input_factory
    )

    x_measured = kst_traj.initial_state

    traj_dict = {0: x_measured}

    for step, x_desired in kst_traj.points.items():

        pid_velocity: PIDController = PIDController(
            kp=1.0,
            ki=0,
            kd=0
        )

        pid_steering_angle: PIDController = PIDController(
            kp=10.0,
            ki=0,
            kd=0
        )

        u_a = pid_velocity.compute_control_input(
            measured_state=x_measured.velocity,
            desired_state=x_desired.velocity,
            controller_time_step=controller_time
        )

        u_dv = pid_steering_angle.compute_control_input(
            measured_state=x_measured.steering_angle,
            desired_state=x_desired.steering_angle,
            controller_time_step=controller_time
        )

        u_now = state_input_factory.input_from_args(
            acceleration=u_a,
            steering_angle_velocity=u_dv
        )

        for control_step in range(int(planner_time/controller_time)):
            x_measured = simulation.simulate(
                x0=x_measured,
                u=u_now,
                time_horizon=controller_time
            )
            traj_dict[step+1] = x_measured

            u_a = pid_velocity.compute_control_input(
                measured_state=x_measured.velocity,
                desired_state=x_desired.velocity,
                controller_time_step=controller_time
            )

            u_dv = pid_steering_angle.compute_control_input(
                measured_state=x_measured.steering_angle,
                desired_state=x_desired.steering_angle,
                controller_time_step=controller_time
            )

            u_now = state_input_factory.input_from_args(
                acceleration=u_a,
                steering_angle_velocity=u_dv
            )


    simulated_traj = state_input_factory.trajectory_from_state_or_input(
        kst_dict=traj_dict,
        mode='input',
        t_0=0,
        delta_t=planner_time
    )

    img_save_path = "/home/tmasc/projects/cr-control/output"

    visualize_trajectories(
        scenario=scenario,
        planning_problem=planning_problem,
        planner_trajectory=kst_traj,
        controller_trajectory=simulated_traj
    )










def execute_planner() -> Tuple[KSTTrajectory, KSTTrajectory]:
    """
    Dummy loading precomputed Reactive Planner KST Trajectory
    :return: kst trajectory for state and input
    """
    input_file = Path(__file__).parents[0] / "test/reactive_planner_traj/ZAM_Tjunction-1_42_T-1/input.txt"
    state_file = Path(__file__).parents[0] / "test/reactive_planner_traj/ZAM_Tjunction-1_42_T-1/state.txt"
    with open(input_file, "r") as f:
        i = [ast.literal_eval(el) for el in f.readlines()]
        rp_inputs: List[InputState] = list()
        for idx, el in enumerate(i):
            rp_inputs.append(InputState(acceleration=el[0], steering_angle_speed=el[1], time_step=idx))

    with open(state_file, "r") as f:
        s = [ast.literal_eval(el.replace("array", '')) for el in f.readlines()]
        rp_states: List[ReactivePlannerState] = list()
        for idx, el in enumerate(s):
            rp_states.append(
                ReactivePlannerState(
                    time_step=idx,
                    position=np.asarray(el[0]),
                    steering_angle=el[1],
                    velocity=el[2],
                    orientation=el[3],
                    acceleration=el[4],
                    yaw_rate=el[5]
                )
            )
    rpc = ReactivePlannerConverter()
    return (
        rpc.trajectory_p2c_kst(planner_traj=rp_states, mode='state'),
        rpc.trajectory_p2c_kst(planner_traj=rp_inputs, mode='input')
    )



if __name__ == "__main__":
    main()