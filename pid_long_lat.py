import copy
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader
from shapely.geometry import LineString, Point

from commonroad_control.control.pid.pid_long_lat import PIDLongLat
from commonroad_control.noise_disturbance.GaussianNDGenerator import GaussianNDGenerator
from commonroad_control.util.geometry import signed_distance_point_to_linestring
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.simulation.simulation import Simulation
from commonroad_control.util.clcs_control_util import extend_kb_reference_trajectory_lane_following
from commonroad_control.util.state_conversion import convert_state_kb2db, convert_state_db2kb
from commonroad_control.util.visualization.visualize_control_state import visualize_reference_vs_actual_states
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories, make_gif
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

    print("initialize params")
    # controller
    dt_controller = 0.1
    look_ahead_s = 0.5

    # vehicle parameters
    vehicle_params = BMW3seriesParams()
    extended_horizon_steps = 10

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

    # Lookahead
    look_ahead_sim: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        state_input_factory=sit_factory_sim
    )

    pid_controller: PIDLongLat = PIDLongLat(
        kp_long=1.0,
        ki_long=0.0,
        kd_long=0.0,
        kp_steer_offset=0.05,
        ki_steer_offset=0.0,
        kd_steer_offset=0.2,
        kp_steer_heading=0.01,
        ki_steer_heading=0.0,
        kd_steer_heading=0.04,
        dt=dt_controller,
    )
    print("run controller")

    # extend reference trajectory
    clcs_traj, x_ref_ext, u_ref_ext = extend_kb_reference_trajectory_lane_following(
        x_ref=copy.copy(x_ref),
        u_ref=copy.copy(u_ref),
        lanelet_network=scenario.lanelet_network,
        vehicle_params=vehicle_params,
        delta_t=dt_controller,
        horizon=extended_horizon_steps)
    reference_trajectory = ReferenceTrajectoryFactory(
        delta_t_controller=dt_controller,
        sit_factory=KBSITFactory(),
        horizon=extended_horizon_steps,
    )
    reference_trajectory.set_reference_trajectory(
        state_ref=x_ref_ext,
        input_ref=u_ref_ext,
        t_0=0
    )
    ref_path: LineString = LineString(
        [(p.position_x, p.position_y) for p in reference_trajectory.state_trajectory.points.values()]
    )


    x_measured = convert_state_kb2db(kb_state=x_ref.initial_point,
                                     vehicle_params=vehicle_params
                                     )


    x_disturbed = copy.copy(x_measured)
    traj_dict_measured = {0: x_measured}
    traj_dict_dist_no_noise = {0: x_disturbed}
    input_dict = {}


    for kk_sim in range(len(x_ref.steps)):
        # extract reference trajectory
        tmp_x_ref, tmp_u_ref = reference_trajectory.get_reference_trajectory_at_time(
            t=kk_sim * dt_controller + look_ahead_s
        )

        u_look_ahead_sim = sit_factory_sim.input_from_args(
            acceleration=u_ref.points[kk_sim].acceleration,
            steering_angle_velocity=u_ref.points[kk_sim].steering_angle_velocity
        )
        _, _, x_look_ahead = look_ahead_sim.simulate(
            x0=x_disturbed,
            u=u_look_ahead_sim,
            time_horizon=look_ahead_s
        )

        # convert simulated forward step state back to KB for control
        x0_kb = convert_state_db2kb(x_look_ahead)
        lateral_offset_lookahead = signed_distance_point_to_linestring(
            point=Point(x0_kb.position_x, x0_kb.position_y),
            linestring=ref_path
        )

        u_vel, u_steer = pid_controller.compute_control_input(
            measured_v_long=x0_kb.velocity,
            desired_v_long=tmp_x_ref.points[0].velocity,
            measured_heading=x0_kb.heading,
            desired_heading=tmp_x_ref.points[0].heading,
            measured_lat_offset=lateral_offset_lookahead,
            desired_lat_offset=0.0
        )

        u_now = sit_factory_sim.input_from_args(
            acceleration=u_vel + u_ref.points[kk_sim].acceleration,
            steering_angle_velocity=u_steer + u_ref.points[kk_sim].steering_angle_velocity
        )


        # simulate
        x_measured, x_disturbed, x_nominal = simulation.simulate(
            x0=x_disturbed,
            u=u_now,
            time_horizon=dt_controller
        )

        # update dicts
        input_dict[kk_sim] = u_now
        traj_dict_measured[kk_sim + 1] = x_measured
        traj_dict_dist_no_noise[kk_sim + 1] = x_disturbed



    print(f"visualization")
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

    visualize_reference_vs_actual_states(
        reference_trajectory=x_ref,
        actual_trajectory=simulated_traj,
        time_steps=list(simulated_traj.points.keys())[:-2],
        save_img=save_imgs,
        save_path=img_save_path
    )

if __name__ == "__main__":
    scenario_name = "DEU_AachenFrankenburg-1_2621353_T-21698"
    # scenario_name = "C-DEU_B471-2_1"
    scenario_file = Path(__file__).parents[0] / "scenarios" / str(scenario_name + ".xml")
    planner_config_path = Path(__file__).parents[0] / "scenarios" / "reactive_planner_configs" / str(scenario_name + ".yaml")
    img_save_path = Path(__file__).parents[0] / "output" / scenario_name


    main(
        scenario_file=scenario_file,
        scenario_name=scenario_name,
        img_save_path=img_save_path,
        planner_config_path=planner_config_path,
        save_imgs=True,
        create_scenario=True
    )
