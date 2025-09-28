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

from commonroad_control.noise_disturbance.GaussianNDGenerator import GaussianNDGenerator
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBStateIndices, KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInputIndices, KBInput

from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.simulation.simulation import Simulation
from commonroad_control.util.clcs_control_util import extend_reference_trajectory_lane_following, extend_kb_reference_trajectory_lane_following
from commonroad_control.util.state_conversion import convert_state_kb2db, convert_state_db2kb
from commonroad_control.util.visualization.visualize_control_state import visualize_desired_vs_actual_states

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from rp_test import main as rpmain


from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories, make_gif

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx, SCvxParameters
from commonroad_clcs.clcs import CurvilinearCoordinateSystem

from commonroad_control.control.reference_trajectory_factory import ReferenceTrajectoryFactory



def main(
        scenario_file: Path,
        scenario_name: str,
        img_save_path: Path,
        planner_config_path: Path,
        save_imgs: bool,
        create_scenario: bool = True
) -> None:
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    print(f"solving scnenario {str(scenario.scenario_id)}")

    # run planner
    print("run planner")
    rp_states, rp_inputs = run_reactive_planner(
        scenario=scenario,
        scenario_xml_file_name=str(scenario_name + ".xml"),
        planning_problem=planning_problem,
        planning_problem_set=planning_problem_set,
        reactive_planner_config_path=planner_config_path
    )
    rpc = ReactivePlannerConverter()
    x_ref = rpc.trajectory_p2c_kb(
        planner_traj=rp_states,
        mode=TrajectoryMode.State
    )
    u_ref = rpc.trajectory_p2c_kb(
        planner_traj=rp_inputs,
        mode=TrajectoryMode.Input
    )

    print("run controller")
    # vehicle parameters
    vehicle_params = BMW3seriesParams()

    # controller parameters
    horizon_ocp = 10
    dt_controller = 0.1
    # ... vehicle model for prediction
    vehicle_model_ctrl = KinematicBicycle(
        params=vehicle_params,
        delta_t=dt_controller)
    sit_factory_ctrl = KBSITFactory()
    # ... initialize optimal control solver
    cost_xx = np.eye(KBStateIndices.dim)
    cost_xx[KBStateIndices.steering_angle, KBStateIndices.steering_angle] = 0.0
    cost_uu = 0.01 * np.eye(KBInputIndices.dim)
    cost_final = np.eye(KBStateIndices.dim)
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

    # simulation
    sit_factory_sim = DBSITFactory()
    vehicle_model_sim = DynamicBicycle(params=vehicle_params, delta_t=dt_controller)
    disturbance_generator: GaussianNDGenerator = GaussianNDGenerator(
        dim=vehicle_model_sim.state_dimension,
        means=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    noise_generator: GaussianNDGenerator = GaussianNDGenerator(
        dim=vehicle_model_sim.state_dimension,
        means=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.075, 0.075, 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        state_input_factory=sit_factory_sim,
        disturbance_generator=disturbance_generator,
        noise_generator=noise_generator
    )

    # extend reference trajectory
    clcs_traj, x_ref_ext, u_ref_ext = extend_kb_reference_trajectory_lane_following(
        x_ref=copy.copy(x_ref),
        u_ref=copy.copy(u_ref),
        lanelet_network=scenario.lanelet_network,
        vehicle_params=vehicle_params,
        delta_t=dt_controller,
        horizon=mpc.horizon)
    reference_trajectory = ReferenceTrajectoryFactory(
        delta_t_controller=dt_controller,
        sit_factory=KBSITFactory(),
        horizon=mpc.horizon,
    )
    reference_trajectory.set_reference_trajectory(
        state_ref=x_ref_ext,
        input_ref=u_ref_ext,
        t_0=0
    )

    x_measured = convert_state_kb2db(kb_state=x_ref.initial_point,
                                     vehicle_params=vehicle_params
                                     )
    # x_measured = x_ref.initial_point

    x_disturbed = copy.copy(x_measured)

    traj_dict_measured = {0: x_measured}
    traj_dict_dist_no_noise = {0: x_disturbed}
    input_dict = {}
    x_ref_steps = [kk for kk in range(mpc.horizon + 1)]
    u_ref_steps = [kk for kk in range(mpc.horizon)]

    # for step, x_planner in kb_traj.points.items():
    for kk_sim in range(len(x_ref.steps)):

        # extract reference trajectory
        tmp_x_ref, tmp_u_ref = reference_trajectory.get_reference_trajectory_at_time(
            t=kk_sim*dt_controller
        )

        # convert initial state to kb
        x0_kb = convert_state_db2kb(traj_dict_measured[kk_sim])
        # x0_kb = traj_dict[kk_sim]

        # compute control input
        u_now = mpc.compute_control_input(
            x0=x0_kb,
            x_ref=tmp_x_ref,
            u_ref=tmp_u_ref
        )
        # u_now = kb_input.points[kk_sim]
        input_dict[kk_sim] = u_now
        # simulate
        x_measured, x_disturbed, x_rk45 = simulation.simulate(
            x0=x_disturbed,
            u=u_now,
            time_horizon=dt_controller
        )
        traj_dict_measured[kk_sim+1] = x_measured
        traj_dict_dist_no_noise[kk_sim + 1] = x_disturbed

    simulated_traj = sit_factory_sim.trajectory_from_state_or_input(
        trajectory_dict=traj_dict_measured,
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


if __name__ == "__main__":
    scenario_name = "ZAM_Over-1_1"
    scenario_file = Path(__file__).parents[0] / "scenarios" / str(scenario_name + ".xml")
    planner_config_path = Path(__file__).parents[0]/ "scenarios" / "reactive_planner_configs" / str(scenario_name + ".yaml")
    img_save_path = Path(__file__).parents[0] / "output" / scenario_name
    main(
        scenario_file=scenario_file,
        scenario_name=scenario_name,
        img_save_path=img_save_path,
        planner_config_path=planner_config_path,
        save_imgs=True,
        create_scenario=True
    )
