import copy
import math
import random
import unittest
import ast
from pathlib import Path


import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_clcs.config import CLCSParams
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
from commonroad_clcs.clcs import CurvilinearCoordinateSystem




def main(
        scenario_file: Path,
        img_save_path: str,
        planner_input_file: Path,
        planner_state_file: Path,
        save_imgs: bool

) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    controller_time: float = 0.01
    planner_time: float = 0.1


    state_input_factory = KSTSITFactory()

    kst_traj, kst_input = execute_planner(
        input_file=planner_input_file,
        state_file=planner_state_file
    )
    vehicle_params: VehicleParameters = BMW3seriesParams()
    vehicle_model: KinematicSingleStrack = KinematicSingleStrack(params=vehicle_params, dt=controller_time)
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model,
        state_input_factory=state_input_factory
    )

    clcs_traj: CurvilinearCoordinateSystem = CurvilinearCoordinateSystem(
        reference_path=np.asarray([[state.position_x, state.position_y] for state in kst_traj.points.values()]),
        params=CLCSParams()
    )

    x_measured = kst_traj.initial_state

    traj_dict = {0: x_measured}

    for step, x_desired in kst_traj.points.items():

        if(step > 140):
            break

        pid_velocity: PIDController = PIDController(
            kp=1.0,
            ki=0.01,
            kd=0
        )

        pid_steering_angle: PIDController = PIDController(
            kp=2.0,
            ki=0.00,
            kd=1.0
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

            current_position_curv = clcs_traj.convert_to_curvilinear_coords(
                x=x_measured.position_x,
                y=x_measured.position_y
            )


            u_a = pid_velocity.compute_control_input(
                measured_state=x_measured.velocity,
                desired_state=x_desired.velocity,
                controller_time_step=controller_time
            )

            u_dv = pid_steering_angle.compute_control_input(
                measured_state=current_position_curv[1],
                desired_state=0.0,
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



    visualize_trajectories(
        scenario=scenario,
        planning_problem=planning_problem,
        planner_trajectory=kst_traj,
        controller_trajectory=simulated_traj,
        save_path=img_save_path,
        save_img=save_imgs
    )

    if save_imgs:
        make_gif(
            path_to_img_dir=img_save_path,
            scenario_name=str(scenario.scenario_id),
            num_imgs=len(kst_traj.points.values())
        )









def execute_planner(
        input_file: Path,
        state_file: Path
) -> Tuple[KSTTrajectory, KSTTrajectory]:
    """
    Dummy loading precomputed Reactive Planner KST Trajectory
    :return: kst trajectory for state and input
    """
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
    scenario_name = "ZAM_Tjunction-1_42_T-1"
    scenario_file = Path(__file__).parents[0] / "scenarios" / str(scenario_name + ".xml")
    planner_input_file = Path(__file__).parents[0] / "test/reactive_planner_traj" / scenario_name / "input.txt"
    planner_state_file = Path(__file__).parents[0] / "test/reactive_planner_traj" / scenario_name / "state.txt"
    img_save_path = "/home/tmasc/projects/cr-control/output"
    main(
        scenario_file=scenario_file,
        img_save_path=img_save_path,
        planner_input_file=planner_input_file,
        planner_state_file=planner_state_file,
        save_imgs=True
    )

