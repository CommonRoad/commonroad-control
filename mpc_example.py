import copy
import math
import random
import unittest
from typing import List, Tuple
import ast
from pathlib import Path
import os

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_clcs.config import CLCSParams
from commonroad_rp.state import ReactivePlannerState
from commonroad.scenario.state import InputState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle

from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTStateIndices, KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInputIndices, KSTInput

from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.simulation.simulation import Simulation
from commonroad_control.util.clcs_control_util import extend_reference_trajectory_lane_following, extend_kst_reference_trajectory_lane_following
from commonroad_control.util.state_conversion import convert_state_kst2dst, convert_state_dst2kst
from commonroad_control.util.visualization.visualize_control_state import visualize_desired_vs_actual_states

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from rp_test import main as rpmain


from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories, make_gif

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx, SCvxParameters
from commonroad_clcs.clcs import CurvilinearCoordinateSystem




def main(
        scenario_file: Path,
        img_save_path: Path,
        planner_input_file: Path,
        planner_state_file: Path,
        save_imgs: bool,
        create_scenario: bool = True
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    print(f"solving scnenario {str(scenario.scenario_id)}")

    # vehicle parameters
    vehicle_params = BMW3seriesParams()

    # controller parameters
    horizon_ocp = 10
    dt_controller = 0.1
    # ... vehicle model for prediction
    vehicle_model_ctrl = KinematicSingleStrack(
        params=vehicle_params,
        delta_t=dt_controller)
    sit_factory_ctrl = KSTSITFactory()
    # ... initialize optimal control solver
    cost_xx = np.eye(KSTStateIndices.dim)
    cost_uu = 0.01 * np.eye(KSTInputIndices.dim)
    cost_final = np.eye(KSTStateIndices.dim)
    # ... real time iteration -> only one iteration per time step
    solver_parameters = SCvxParameters(max_iterations=1)
    scvx_solver = OptimalControlSCvx(
        vehicle_model=vehicle_model_ctrl,
        sit_factory=sit_factory_ctrl,
        horizon=horizon_ocp,
        delta_t=dt_controller,
        cost_xx=cost_xx,
        cost_uu=cost_uu,
        cost_final=cost_final,
        ocp_parameters=solver_parameters
    )

    # instantiate model predictive controller
    mpc = ModelPredictiveControl(
        ocp_solver=scvx_solver
    )

    x_ref, u_ref = execute_planner(
        input_file=planner_input_file,
        state_file=planner_state_file
    )

    # simulation
    sit_factory_sim = DBSITFactory()
    vehicle_model_sim = DynamicBicycle(params=vehicle_params, delta_t=dt_controller)
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        state_input_factory=sit_factory_sim
    )

    # extend reference trajectory
    clcs_traj, x_ref_ext, u_ref_ext = extend_kst_reference_trajectory_lane_following(
        x_ref=copy.copy(x_ref),
        u_ref=copy.copy(u_ref),
        lanelet_network=scenario.lanelet_network,
        vehicle_params=vehicle_params,
        delta_t=dt_controller,
        horizon=mpc.horizon)

    x_measured = convert_state_kst2dst(kst_state=x_ref.initial_point,
                                       vehicle_params=vehicle_params
                                       )
    # x_measured = x_ref.initial_point

    traj_dict = {0: x_measured}
    input_dict = {}
    x_ref_steps = [kk for kk in range(mpc.horizon + 1)]
    u_ref_steps = [kk for kk in range(mpc.horizon)]

    # for step, x_planner in kst_traj.points.items():
    for kk_sim in range(len(x_ref.steps)):

        # extract reference trajectory
        tmp_x_ref_points = [x_ref_ext.points[kk+kk_sim] for kk in x_ref_steps]
        tmp_x_ref = sit_factory_ctrl.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(x_ref_steps,tmp_x_ref_points)),
            mode=TrajectoryMode.State,
            t_0=kk_sim * dt_controller,
            delta_t=dt_controller
        )
        tmp_u_ref_points = [u_ref_ext.points[kk+kk_sim] for kk in u_ref_steps]
        tmp_u_ref = sit_factory_ctrl.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(u_ref_steps,tmp_u_ref_points)),
            mode=TrajectoryMode.Input,
            t_0=kk_sim * dt_controller,
            delta_t=dt_controller
        )

        # convert initial state to kst
        x0_kst = convert_state_dst2kst(traj_dict[kk_sim])
        # x0_kst = traj_dict[kk_sim]

        # compute control input
        u_now = mpc.compute_control_input(
            x0=x0_kst,
            x_ref=tmp_x_ref,
            u_ref=tmp_u_ref
        )
        # u_now = kst_input.points[kk_sim]
        input_dict[kk_sim] = u_now
        # simulate
        x_measured = simulation.simulate(
            x0=x_measured,
            u=u_now,
            time_horizon=dt_controller
        )
        traj_dict[kk_sim+1] = x_measured




    # simulated_traj = state_input_factory.trajectory_from_state_or_input(
    #     trajectory_dict=traj_dict,
    #     mode=TrajectoryMode.Input,
    #     t_0=0,
    #     delta_t=dt_controller
    # )
    simulated_traj = sit_factory_sim.trajectory_from_state_or_input(
        trajectory_dict=traj_dict,
        mode=TrajectoryMode.State,
        t_0=0,
        delta_t=dt_controller
    )


    if create_scenario:
        visualize_trajectories(
            scenario=scenario,
            planning_problem=planning_problem,
            planner_trajectory=x_ref,
            controller_trajectory=simulated_traj,
            save_path=img_save_path,
            save_img=save_imgs
        )

        if save_imgs:
            make_gif(
                path_to_img_dir=img_save_path,
                scenario_name=str(scenario.scenario_id),
                num_imgs=len(x_ref.points.values())
            )

    visualize_desired_vs_actual_states(
        desired_states=x_ref,
        actual_states=simulated_traj,
        time_steps=list(simulated_traj.points.keys())[:-2],
        state_dim=x_ref.dim,
        save_img=save_imgs,
        save_path=img_save_path
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
        rpc.trajectory_p2c_kst(
            planner_traj=rp_states,
            mode=TrajectoryMode.State),
        rpc.trajectory_p2c_kst(
            planner_traj=rp_inputs,
            mode=TrajectoryMode.Input)
    )



if __name__ == "__main__":
    scenario_name = "ZAM_Over-1_1"
    scenario_file = Path(__file__).parents[0] / "scenarios" / str(scenario_name + ".xml")
    planner_input_file = Path(__file__).parents[0] / "test/reactive_planner_traj" / scenario_name / "input.txt"
    planner_state_file = Path(__file__).parents[0] / "test/reactive_planner_traj" / scenario_name / "state.txt"
    img_save_path = Path(__file__).parents[0] / "output" / scenario_name
    main(
        scenario_file=scenario_file,
        img_save_path=img_save_path,
        planner_input_file=planner_input_file,
        planner_state_file=planner_state_file,
        save_imgs=True,
        create_scenario=True
    )
