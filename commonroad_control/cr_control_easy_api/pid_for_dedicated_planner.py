import copy
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Type, Union

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState
from scipy.integrate import OdeSolver
from shapely.geometry import LineString, Point

from commonroad_control.control.pid.pid_long_lat import PIDLongLat
from commonroad_control.control.reference_trajectory_factory import (
    ReferenceTrajectoryFactory,
)
from commonroad_control.planning_converter.planning_converter_interface import (
    PlanningConverterInterface,
)
from commonroad_control.planning_converter.reactive_planner_converter import (
    ReactivePlannerConverter,
)
from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import (
    FullStateFeedback,
)
from commonroad_control.simulation.sensor_models.sensor_model_interface import (
    SensorModelInterface,
)
from commonroad_control.simulation.simulation.simulation import Simulation
from commonroad_control.simulation.uncertainty_model.gaussian_distribution import (
    GaussianDistribution,
)
from commonroad_control.simulation.uncertainty_model.no_uncertainty import NoUncertainty
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)
from commonroad_control.util.clcs_control_util import (
    extend_kb_reference_trajectory_lane_following,
)
from commonroad_control.util.geometry import signed_distance_point_to_linestring
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import (
    run_reactive_planner,
)
from commonroad_control.util.state_conversion import (
    convert_state_db2kb,
    convert_state_kb2db,
)
from commonroad_control.util.visualization.visualize_control_state import (
    visualize_reference_vs_actual_states,
)
from commonroad_control.util.visualization.visualize_trajectories import (
    make_gif,
    visualize_trajectories,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import (
    DBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import (
    DynamicBicycle,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import (
    KBSITFactoryDisturbance,
)
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters

logger = logging.getLogger(__name__)


def pid_with_lookahead_for_reactive_planner_no_uncertainty(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    reactive_planner_state_trajectory: List[ReactivePlannerState],
    reactive_planner_input_trajectory: List[InputState],
    kp_long: float = 1.0,
    ki_long: float = 0.0,
    kd_long: float = 0.0,
    kp_steer_offset: float = 0.05,
    ki_steer_offset: float = 0.0,
    kd_steer_offset: float = 0.1,
    dt_controller: float = 0.1,
    look_ahead_s: float = 0.5,
    extended_horizon_steps: int = 10,
    vehicle_params=BMW3seriesParams(),
    planner_converter: Optional[
        PlanningConverterInterface
    ] = ReactivePlannerConverter(),
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_typename: Type[VehicleModelInterface] = DynamicBicycle,
    sensor_model_typename: Type[
        Union[SensorModelInterface, FullStateFeedback]
    ] = FullStateFeedback,
    func_convert_planner2controller_state: Callable[
        [StateInterface, VehicleParameters], StateInterface
    ] = convert_state_kb2db,
    func_convert_controller2planner_state: Callable[
        [StateInterface], StateInterface
    ] = convert_state_db2kb,
    ivp_method: Union[str, OdeSolver, None] = "RK45",
    visualize_scenario: bool = False,
    visualize_control: bool = False,
    save_imgs: bool = False,
    img_saving_path: Union[Path, str] = None,
) -> Tuple[
    Dict[int, StateInterface], Dict[int, StateInterface], Dict[int, InputInterface]
]:
    return pid_with_lookahead_for_reactive_planner(
        scenario=scenario,
        planning_problem=planning_problem,
        reactive_planner_state_trajectory=reactive_planner_state_trajectory,
        reactive_planner_input_trajectory=reactive_planner_input_trajectory,
        kp_long=kp_long,
        ki_long=ki_long,
        kd_long=kd_long,
        kp_steer_offset=kp_steer_offset,
        ki_steer_offset=ki_steer_offset,
        kd_steer_offset=kd_steer_offset,
        dt_controller=dt_controller,
        look_ahead_s=look_ahead_s,
        extended_horizon_steps=extended_horizon_steps,
        vehicle_params=vehicle_params,
        planner_converter=planner_converter,
        disturbance_model_typename=NoUncertainty,
        noise_model_typename=NoUncertainty,
        sit_factory_sim=sit_factory_sim,
        vehicle_model_typename=vehicle_model_typename,
        sensor_model_typename=sensor_model_typename,
        func_convert_planner2controller_state=func_convert_planner2controller_state,
        func_convert_controller2planner_state=func_convert_controller2planner_state,
        ivp_method=ivp_method,
        visualize_scenario=visualize_scenario,
        visualize_control=visualize_control,
        save_imgs=save_imgs,
        img_saving_path=img_saving_path,
    )


def pid_with_lookahead_for_reactive_planner(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    reactive_planner_state_trajectory: List[ReactivePlannerState],
    reactive_planner_input_trajectory: List[InputState],
    kp_long: float = 1.0,
    ki_long: float = 0.0,
    kd_long: float = 0.0,
    kp_steer_offset: float = 0.05,
    ki_steer_offset: float = 0.0,
    kd_steer_offset: float = 0.1,
    dt_controller: float = 0.1,
    look_ahead_s: float = 0.5,
    extended_horizon_steps: int = 10,
    vehicle_params=BMW3seriesParams(),
    planner_converter: Union[
        PlanningConverterInterface, ReactivePlannerConverter
    ] = ReactivePlannerConverter(),
    disturbance_model_typename: Type[UncertaintyModelInterface] = GaussianDistribution,
    noise_model_typename: Type[UncertaintyModelInterface] = GaussianDistribution,
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_typename: Type[VehicleModelInterface] = DynamicBicycle,
    sensor_model_typename: Type[
        Union[SensorModelInterface, FullStateFeedback]
    ] = FullStateFeedback,
    func_convert_planner2controller_state: Callable[
        [StateInterface, VehicleParameters], StateInterface
    ] = convert_state_kb2db,
    func_convert_controller2planner_state: Callable[
        [StateInterface], StateInterface
    ] = convert_state_db2kb,
    ivp_method: Union[str, OdeSolver, None] = "RK45",
    visualize_scenario: bool = False,
    visualize_control: bool = False,
    save_imgs: bool = False,
    img_saving_path: Union[Path, str] = None,
) -> Tuple[
    Dict[int, StateInterface], Dict[int, StateInterface], Dict[int, InputInterface]
]:
    return pid_with_lookahead_for_planner(
        scenario=scenario,
        planning_problem=planning_problem,
        state_trajectory=reactive_planner_state_trajectory,
        input_trajectory=reactive_planner_input_trajectory,
        kp_long=kp_long,
        ki_long=ki_long,
        kd_long=kd_long,
        kp_steer_offset=kp_steer_offset,
        ki_steer_offset=ki_steer_offset,
        kd_steer_offset=kd_steer_offset,
        dt_controller=dt_controller,
        look_ahead_s=look_ahead_s,
        extended_horizon_steps=extended_horizon_steps,
        vehicle_params=vehicle_params,
        planner_converter=planner_converter,
        disturbance_model_typename=disturbance_model_typename,
        noise_model_typename=noise_model_typename,
        sit_factory_sim=sit_factory_sim,
        vehicle_model_typename=vehicle_model_typename,
        sensor_model_typename=sensor_model_typename,
        func_convert_planner2controller_state=func_convert_planner2controller_state,
        func_convert_controller2planner_state=func_convert_controller2planner_state,
        ivp_method=ivp_method,
        visualize_scenario=visualize_scenario,
        visualize_control=visualize_control,
        save_imgs=save_imgs,
        img_saving_path=img_saving_path,
    )


def pid_with_lookahead_for_planner(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    state_trajectory: Any,
    input_trajectory: Any,
    planner_converter: Union[ReactivePlannerConverter, PlanningConverterInterface],
    kp_long: float = 1.0,
    ki_long: float = 0.0,
    kd_long: float = 0.0,
    kp_steer_offset: float = 0.05,
    ki_steer_offset: float = 0.0,
    kd_steer_offset: float = 0.1,
    dt_controller: float = 0.1,
    look_ahead_s: float = 0.5,
    extended_horizon_steps: int = 10,
    vehicle_params=BMW3seriesParams(),
    disturbance_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    noise_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_typename: Type[VehicleModelInterface] = DynamicBicycle,
    sensor_model_typename: Type[
        Union[SensorModelInterface, FullStateFeedback]
    ] = FullStateFeedback,
    func_convert_planner2controller_state: Callable[
        [StateInterface, VehicleParameters], StateInterface
    ] = convert_state_kb2db,
    func_convert_controller2planner_state: Callable[
        [StateInterface], StateInterface
    ] = convert_state_db2kb,
    ivp_method: Union[str, OdeSolver, None] = "RK45",
    visualize_scenario: bool = False,
    visualize_control: bool = False,
    save_imgs: bool = False,
    img_saving_path: Union[Path, str] = None,
) -> Tuple[
    Dict[int, StateInterface], Dict[int, StateInterface], Dict[int, InputInterface]
]:

    x_ref = planner_converter.trajectory_p2c_kb(
        planner_traj=state_trajectory, mode=TrajectoryMode.State
    )
    u_ref = planner_converter.trajectory_p2c_kb(
        planner_traj=input_trajectory, mode=TrajectoryMode.Input
    )

    logger.debug("initialize simulation")
    # simulation
    # ... vehicle model
    vehicle_model_sim = vehicle_model_typename(
        params=vehicle_params, delta_t=dt_controller
    )
    # ... disturbances
    sim_disturbance_model = disturbance_model_typename(
        dim=vehicle_model_sim.disturbance_dimension,
        mean=vehicle_params.disturbance_gaussian_mean,
        std_deviation=vehicle_params.disturbance_gaussian_std,
    )
    # ... noise
    sim_noise_model = noise_model_typename(
        dim=vehicle_model_sim.disturbance_dimension,
        mean=vehicle_params.noise_gaussian_mean,
        std_deviation=vehicle_params.noise_gaussian_std,
    )
    # ... sensor model
    sensor_model = sensor_model_typename(
        noise_model=sim_noise_model,
        state_output_factory=sit_factory_sim,
        state_dimension=sit_factory_sim.state_dimension,
        input_dimension=sit_factory_sim.input_dimension,
    )
    # ... simulation
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        state_input_factory=sit_factory_sim,
        disturbance_model=sim_disturbance_model,
        random_disturbance=True,
        sensor_model=sensor_model,
        random_noise=True,
        delta_t_sim=dt_controller,
    )

    # Lookahead
    # ... simulation
    look_ahead_sim: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        state_input_factory=sit_factory_sim,
    )

    pid_controller: PIDLongLat = PIDLongLat(
        kp_long=kp_long,
        ki_long=ki_long,
        kd_long=kd_long,
        kp_steer_offset=kp_steer_offset,
        ki_steer_offset=ki_steer_offset,
        kd_steer_offset=kd_steer_offset,
        dt=dt_controller,
    )

    logger.debug("run controller")
    t_0 = time.perf_counter()
    eta: float = 0.0

    # extend reference trajectory
    clcs_traj, x_ref_ext, u_ref_ext = extend_kb_reference_trajectory_lane_following(
        x_ref=copy.copy(x_ref),
        u_ref=copy.copy(u_ref),
        lanelet_network=scenario.lanelet_network,
        vehicle_params=vehicle_params,
        delta_t=dt_controller,
        horizon=extended_horizon_steps,
    )
    reference_trajectory = ReferenceTrajectoryFactory(
        delta_t_controller=dt_controller,
        sit_factory=KBSITFactoryDisturbance(),
        horizon=extended_horizon_steps,
    )
    reference_trajectory.set_reference_trajectory(
        state_ref=x_ref_ext, input_ref=u_ref_ext, t_0=0
    )
    ref_path: LineString = LineString(
        [
            (p.position_x, p.position_y)
            for p in reference_trajectory.state_trajectory.points.values()
        ]
    )

    x_measured = func_convert_planner2controller_state(
        kb_state=x_ref.initial_point, vehicle_params=vehicle_params
    )

    x_disturbed = copy.copy(x_measured)
    traj_dict_measured = {0: x_measured}
    traj_dict_no_noise = {0: x_disturbed}
    input_dict = {}

    for kk_sim in range(len(x_ref.steps)):
        # extract reference trajectory
        tmp_x_ref, tmp_u_ref = reference_trajectory.get_reference_trajectory_at_time(
            t=kk_sim * dt_controller + look_ahead_s
        )

        u_look_ahead_sim = sit_factory_sim.input_from_args(
            acceleration=u_ref.points[kk_sim].acceleration,
            steering_angle_velocity=u_ref.points[kk_sim].steering_angle_velocity,
        )

        eta = eta + time.perf_counter() - t_0
        _, _, x_look_ahead = look_ahead_sim.simulate(
            x0=traj_dict_no_noise[kk_sim],
            u=u_look_ahead_sim,
            t_final=look_ahead_s,
            ivp_method=ivp_method,
        )
        t_0 = time.perf_counter()

        # convert simulated forward step state back to KB for control
        x0_kb = func_convert_controller2planner_state(x_look_ahead.final_point)
        lateral_offset_lookahead = signed_distance_point_to_linestring(
            point=Point(x0_kb.position_x, x0_kb.position_y), linestring=ref_path
        )

        u_vel, u_steer = pid_controller.compute_control_input(
            measured_v_long=x0_kb.velocity,
            desired_v_long=tmp_x_ref.points[0].velocity,
            measured_lat_offset=lateral_offset_lookahead,
            desired_lat_offset=0.0,
        )

        u_now = sit_factory_sim.input_from_args(
            acceleration=u_vel + u_ref.points[kk_sim].acceleration,
            steering_angle_velocity=u_steer
            + u_ref.points[kk_sim].steering_angle_velocity,
        )

        eta = eta + time.perf_counter() - t_0
        t_0 = time.perf_counter()
        # simulate
        x_measured, x_disturbed, x_nominal = simulation.simulate(
            x0=traj_dict_no_noise[kk_sim],
            u=u_now,
            t_final=dt_controller,
            ivp_method=ivp_method,
        )

        # update dicts
        input_dict[kk_sim] = u_now
        traj_dict_measured[kk_sim + 1] = x_measured
        traj_dict_no_noise[kk_sim + 1] = x_disturbed.final_point

    logger.info(f"Control took: {eta * 1000} millisec.")
    simulated_traj = sit_factory_sim.trajectory_from_state_or_input(
        trajectory_dict=traj_dict_measured,
        mode=TrajectoryMode.State,
        t_0=0,
        delta_t=dt_controller,
    )

    if visualize_scenario:
        logger.info("visualization")
        visualize_trajectories(
            scenario=scenario,
            planning_problem=planning_problem,
            planner_trajectory=x_ref,
            controller_trajectory=simulated_traj,
            save_path=img_saving_path,
            save_img=save_imgs,
        )

        if save_imgs:
            make_gif(
                path_to_img_dir=img_saving_path,
                scenario_name=str(scenario.scenario_id),
                num_imgs=len(x_ref.points.values()),
            )

    if visualize_control:
        visualize_reference_vs_actual_states(
            reference_trajectory=x_ref,
            actual_trajectory=simulated_traj,
            time_steps=list(simulated_traj.points.keys())[:-2],
            save_img=save_imgs,
            save_path=img_saving_path,
        )

    return traj_dict_measured, traj_dict_no_noise, input_dict


if __name__ == "__main__":
    scenario_name = "DEU_AachenFrankenburg-1_2621353_T-21698"
    scenario_file = (
        Path(__file__).parents[2] / "scenarios" / str(scenario_name + ".xml")
    )
    planner_config_path = (
        Path(__file__).parents[2]
        / "scenarios"
        / "reactive_planner_configs"
        / str(scenario_name + ".yaml")
    )
    img_save_path = Path(__file__).parents[2] / "output" / scenario_name

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
        reactive_planner_config_path=planner_config_path,
    )

    noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner(
        scenario=scenario,
        planning_problem=planning_problem,
        reactive_planner_state_trajectory=rp_states,
        reactive_planner_input_trajectory=rp_inputs,
        visualize_scenario=True,
        visualize_control=True,
        save_imgs=False,
        img_saving_path=img_save_path,
    )
