import copy
from pathlib import Path
import logging

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import KBSIDTFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBStateIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInputIndices

from commonroad_control.simulation.simulation.simulation import Simulation
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.simulation.uncertainty_model.gaussian_distribution import GaussianDistribution
from commonroad_control.simulation.sensor_models.sensor_model_interface import SensorModelInterface
from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import FullStateFeedback

from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner
from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.util.clcs_control_util import extend_kb_reference_trajectory_lane_following
from commonroad_control.util.state_conversion import convert_state_kb2db, convert_state_db2kb
from commonroad_control.util.visualization.visualize_control_state import visualize_reference_vs_actual_states
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories, make_gif

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx, SCvxParameters
from commonroad_control.control.reference_trajectory_factory import ReferenceTrajectoryFactory

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

logger_global = configure_toolbox_logging(level=logging.INFO)
logger = logging.getLogger(__name__)

def main(
        scenario_file: Path,
        scenario_name: str,
        img_save_path: Path,
        planner_config_path: Path,
        save_imgs: bool,
        create_scenario: bool = True
) -> None:
    """
    Example function for building a combined MCP controller for the CommonRoad reactive planner.
    :param scenario_file: Path to scenario file
    :param scenario_name: Scenario name
    :param img_save_path: Path to save images to
    :param planner_config_path: Reactive planner config
    :param save_imgs: If true and img_save_path is valid, save imgs to this path
    :param create_scenario: Needs to be true for any visualization to happen
    """
    scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

    logger.info(f"solving scnenario {str(scenario.scenario_id)}")

    # run planner
    logger.info("run planner")
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

    logger.info("run controller")
    # vehicle parameters
    vehicle_params = BMW3seriesParams()

    # controller parameters
    horizon_ocp = 25
    dt_controller = 0.1
    # ... vehicle model for prediction
    vehicle_model_ctrl = KinematicBicycle(
        params=vehicle_params,
        delta_t=dt_controller)
    sit_factory_ctrl = KBSIDTFactory()
    # ... initialize optimal control solver
    cost_xx = np.eye(KBStateIndices.dim)
    cost_xx[KBStateIndices.steering_angle, KBStateIndices.steering_angle] = 0.0
    cost_uu = 0.1 * np.eye(KBInputIndices.dim)
    cost_final = cost_xx  #np.eye(KBStateIndices.dim)
    # ... solver parameters for initial step-> iterate until convergence
    solver_parameters_init = SCvxParameters(max_iterations=20)
    # ... solver parameters for real time iteration -> only one iteration per time step
    solver_parameters_rti = SCvxParameters(max_iterations=1)
    # ... ocp solver (initial parameters)
    scvx_solver = OptimalControlSCvx(
        vehicle_model=vehicle_model_ctrl,
        sidt_factory=sit_factory_ctrl,
        horizon=horizon_ocp,
        delta_t=dt_controller,
        cost_xx=cost_xx,
        cost_uu=cost_uu,
        cost_final=cost_final,
        ocp_parameters=solver_parameters_init
    )
    # instantiate model predictive controller
    mpc = ModelPredictiveControl(
        ocp_solver=scvx_solver
    )

    # simulation
    # ... disturbance model
    sit_factory_sim = DBSIDTFactory()
    vehicle_model_sim = DynamicBicycle(params=vehicle_params, delta_t=dt_controller)
    sim_disturbance_model: UncertaintyModelInterface = GaussianDistribution(
        dim=vehicle_model_sim.state_dimension,
        mean=vehicle_params.disturbance_gaussian_mean,
        std_deviation=vehicle_params.disturbance_gaussian_std
    )
    # ... noise
    sim_noise_model: UncertaintyModelInterface = GaussianDistribution(
        dim=vehicle_model_sim.disturbance_dimension,
        mean=vehicle_params.noise_gaussian_mean,
        std_deviation=vehicle_params.noise_gaussian_std
    )
    # ... sensor model
    sensor_model: SensorModelInterface = FullStateFeedback(
        noise_model=sim_noise_model,
        state_output_factory=sit_factory_sim,
        state_dimension=sit_factory_sim.state_dimension,
        input_dimension=sit_factory_sim.input_dimension
    )
    # ... simulation
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        sidt_factory=sit_factory_sim,
        disturbance_model=sim_disturbance_model,
        random_disturbance=True,
        sensor_model=sensor_model,
        random_noise=True
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
        sidt_factory=KBSIDTFactory(),
        mpc_horizon=mpc.horizon,
    )
    # ... dummy reference trajectory: all inputs set to zero
    u_np = np.zeros((u_ref_ext.dim,len(u_ref_ext.steps)))
    u_ref_0 = sit_factory_ctrl.trajectory_from_numpy_array(
        traj_np=u_np,
        mode=TrajectoryMode.Input,
        time_steps=u_ref_ext.steps,
        t_0=u_ref_ext.t_0,
        delta_t=u_ref_ext.delta_t
    )
    reference_trajectory.set_reference_trajectory(
        state_ref=x_ref_ext,
        input_ref=u_ref_0,
        t_0=0
    )

    # simulation results
    x_measured = convert_state_kb2db(
        kb_state=x_ref.initial_point,
        vehicle_params=vehicle_params
    )
    x_disturbed = copy.copy(x_measured)
    traj_dict_measured = {0: x_measured}
    traj_dict_no_noise = {0: x_disturbed}
    input_dict = {}

    # for step, x_planner in kb_traj.points.items():
    for kk_sim in range(len(u_ref.steps)):

        # extract reference trajectory
        tmp_x_ref, tmp_u_ref = reference_trajectory.get_reference_trajectory_at_time(
            t=kk_sim*dt_controller
        )

        # convert initial state to kb
        x0_kb = convert_state_db2kb(traj_dict_measured[kk_sim])

        # compute control input
        u_now = mpc.compute_control_input(
            x0=x0_kb,
            x_ref=tmp_x_ref,
            u_ref=tmp_u_ref
        )
        if kk_sim == 0:
            # at the initial step: iterate until convergence
            # afterwards: real-time iteration
            mpc.ocp_solver.reset_ocp_parameters(
                new_ocp_parameters=solver_parameters_rti
            )

        # u_now = kb_input.points[kk_sim]
        input_dict[kk_sim] = u_now
        # simulate
        x_measured, x_disturbed, x_nominal = simulation.simulate(
            x0=traj_dict_no_noise[kk_sim],
            u=u_now,
            t_final=dt_controller
        )
        traj_dict_measured[kk_sim+1] = x_measured
        traj_dict_no_noise[kk_sim + 1] = x_disturbed.final_point

    simulated_traj = sit_factory_sim.trajectory_from_points(
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
            save_img=save_imgs,
        )

        if save_imgs:
            make_gif(
                path_to_img_dir=img_save_path,
                scenario_name=str(scenario.scenario_id),
                num_imgs=len(x_ref.points.values())
            )

    visualize_reference_vs_actual_states(
        reference_trajectory=x_ref_ext,
        actual_trajectory=simulated_traj,
        time_steps=list(simulated_traj.points.keys()),
        save_img=save_imgs,
        save_path=img_save_path
    )


if __name__ == "__main__":
    scenario_name = "ITA_Foggia-6_1_T-1"
    # scenario_name = "DEU_AachenFrankenburg-1_2621353_T-21698"
    # scenario_name = "C-DEU_B471-2_1"
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
