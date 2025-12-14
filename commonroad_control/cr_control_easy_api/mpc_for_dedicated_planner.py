import copy
import logging
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple, Type, Union

import numpy as np
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState
from scipy.integrate import OdeSolver

from commonroad_control.control.model_predictive_control.model_predictive_control import (
    ModelPredictiveControl,
)
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import (
    OptimalControlSCvx,
    SCvxParameters,
)
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
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import (
    DBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBStateIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import (
    DynamicBicycle,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import (
    KBInputIndices,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import (
    KBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import (
    KBStateIndices,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import (
    KinematicBicycle,
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


def mpc_for_reactive_planner_no_uncertainty(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    reactive_planner_state_trajectory: List[ReactivePlannerState],
    reactive_planner_input_trajectory: List[InputState],
    planner_converter: Optional[
        PlanningConverterInterface
    ] = ReactivePlannerConverter(),
    dt_controller: float = 0.1,
    horizon_ocp: int = 25,
    vehicle_params=BMW3seriesParams(),
    sit_factory_control: StateInputDisturbanceTrajectoryFactoryInterface = KBSIDTFactory(),
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_control_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = KinematicBicycle,
    vehicle_model_sim_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = DynamicBicycle,
    state_idxs_typename: Union[KBStateIndices, DBStateIndices] = KBStateIndices,
    input_idxs_typename: Union[KBInputIndices, DBInputIndices] = KBInputIndices,
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
    """
    Combined MPC controller for the CommonRoad reactive planner without noise and disturbances.
    Wrapper around mpc_for_planner().
    :param scenario: CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    :param reactive_planner_state_trajectory: CommonRoad reactive planner state trajectory
    :param reactive_planner_input_trajectory: CommonRoad reactive planner input trajectory
    :param planner_converter: CommonRoad reactive planner converter
    :param dt_controller: controller time step size in seconds
    :param horizon_ocp: horizon for the optimal control problem in steps
    :param vehicle_params: vehicle parameters object
    :param sit_factory_control: StateInputTrajectory factory for a given dynamics model used in the sim
    :param sit_factory_sim: StateInputTrajectory factory for a given dynamics model used for control
    :param vehicle_model_control_typename: typename (=class) of the vehicle dynamics model used in control
    :param vehicle_model_sim_typename: typename (=class) of the vehicle dynamics model used in sim
    :param state_idxs_typename: typename (=class) of the state idxs, mapping semantic meaning to an idx
    :param input_idxs_typename: typename (=class) of the input idxs, mapping semantic meaning to an idx
    :param sensor_model_typename: typename (=class) of the sensor model / state feedback
    :param func_convert_planner2controller_state: function to convert a planner state into a controller state
    :param func_convert_controller2planner_state: function to convert a controller state into a planner state
    :param ivp_method: IVP-Method used by the ODE-Solver
    :param visualize_scenario: If true, visualizes the scenario
    :param visualize_control: If true, visualizes control and error outputs
    :param save_imgs: If true and img_saving_path is given, saves visualizations instead of displaying them
    :param img_saving_path: If given and save_imgs=true, saves visualizations instead of displaying them
    :return: measured trajectory, trajectory without noise, trajectory without noise and without disturbance
    """
    return mpc_for_planner(
        scenario=scenario,
        planning_problem=planning_problem,
        state_trajectory=reactive_planner_state_trajectory,
        input_trajectory=reactive_planner_input_trajectory,
        planner_converter=planner_converter,
        dt_controller=dt_controller,
        horizon_ocp=horizon_ocp,
        vehicle_params=vehicle_params,
        disturbance_model_typename=NoUncertainty,
        noise_model_typename=NoUncertainty,
        sit_factory_control=sit_factory_control,
        sit_factory_sim=sit_factory_sim,
        vehicle_model_control_typename=vehicle_model_control_typename,
        vehicle_model_sim_typename=vehicle_model_sim_typename,
        state_idxs_typename=state_idxs_typename,
        input_idxs_typename=input_idxs_typename,
        sensor_model_typename=sensor_model_typename,
        func_convert_planner2controller_state=func_convert_planner2controller_state,
        func_convert_controller2planner_state=func_convert_controller2planner_state,
        ivp_method=ivp_method,
        visualize_scenario=visualize_scenario,
        visualize_control=visualize_control,
        save_imgs=save_imgs,
        img_saving_path=img_saving_path,
    )


def mpc_for_reactive_planner(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    reactive_planner_state_trajectory: List[ReactivePlannerState],
    reactive_planner_input_trajectory: List[InputState],
    planner_converter: Optional[
        PlanningConverterInterface
    ] = ReactivePlannerConverter(),
    dt_controller: float = 0.1,
    horizon_ocp: int = 20,
    vehicle_params=BMW3seriesParams(),
    disturbance_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    noise_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    sit_factory_control: StateInputDisturbanceTrajectoryFactoryInterface = KBSIDTFactory(),
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_control_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = KinematicBicycle,
    vehicle_model_sim_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = DynamicBicycle,
    state_idxs_typename: Union[KBStateIndices, DBStateIndices] = KBStateIndices,
    input_idxs_typename: Union[KBInputIndices, DBInputIndices] = KBInputIndices,
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
    """
    Combined MPC controller for the CommonRoad reactive planner. Wrapper around mpc_for_planner().
    :param scenario: CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    :param reactive_planner_state_trajectory: CommonRoad reactive planner state trajectory
    :param reactive_planner_input_trajectory: CommonRoad reactive planner input trajectory
    :param planner_converter: CommonRoad reactive planner converter
    :param dt_controller: controller time step size in seconds
    :param horizon_ocp: horizon for the optimal control problem in steps
    :param vehicle_params: vehicle parameters object
    :param disturbance_model_typename: typename (=class) of the disturbance
    :param noise_model_typename: typename (=class) of the noise
    :param sit_factory_control: StateInputTrajectory factory for a given dynamics model used in the sim
    :param sit_factory_sim: StateInputTrajectory factory for a given dynamics model used for control
    :param vehicle_model_control_typename: typename (=class) of the vehicle dynamics model used in control
    :param vehicle_model_sim_typename: typename (=class) of the vehicle dynamics model used in sim
    :param state_idxs_typename: typename (=class) of the state idxs, mapping semantic meaning to an idx
    :param input_idxs_typename: typename (=class) of the input idxs, mapping semantic meaning to an idx
    :param sensor_model_typename: typename (=class) of the sensor model / state feedback
    :param func_convert_planner2controller_state: function to convert a planner state into a controller state
    :param func_convert_controller2planner_state: function to convert a controller state into a planner state
    :param ivp_method: IVP-Method used by the ODE-Solver
    :param visualize_scenario: If true, visualizes the scenario
    :param visualize_control: If true, visualizes control and error outputs
    :param save_imgs: If true and img_saving_path is given, saves visualizations instead of displaying them
    :param img_saving_path: If given and save_imgs=true, saves visualizations instead of displaying them
    :return: measured trajectory, trajectory without noise, trajectory without noise and without disturbance
    """
    return mpc_for_planner(
        scenario=scenario,
        planning_problem=planning_problem,
        state_trajectory=reactive_planner_state_trajectory,
        input_trajectory=reactive_planner_input_trajectory,
        planner_converter=planner_converter,
        dt_controller=dt_controller,
        horizon_ocp=horizon_ocp,
        vehicle_params=vehicle_params,
        disturbance_model_typename=disturbance_model_typename,
        noise_model_typename=noise_model_typename,
        sit_factory_control=sit_factory_control,
        sit_factory_sim=sit_factory_sim,
        vehicle_model_control_typename=vehicle_model_control_typename,
        vehicle_model_sim_typename=vehicle_model_sim_typename,
        state_idxs_typename=state_idxs_typename,
        input_idxs_typename=input_idxs_typename,
        sensor_model_typename=sensor_model_typename,
        func_convert_planner2controller_state=func_convert_planner2controller_state,
        func_convert_controller2planner_state=func_convert_controller2planner_state,
        ivp_method=ivp_method,
        visualize_scenario=visualize_scenario,
        visualize_control=visualize_control,
        save_imgs=save_imgs,
        img_saving_path=img_saving_path,
    )


def mpc_for_planner(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    state_trajectory: Any,
    input_trajectory: Any,
    planner_converter: Union[ReactivePlannerConverter, PlanningConverterInterface],
    dt_controller: float = 0.1,
    horizon_ocp: int = 20,
    vehicle_params=BMW3seriesParams(),
    disturbance_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    noise_model_typename: Type[
        Union[UncertaintyModelInterface, GaussianDistribution]
    ] = GaussianDistribution,
    sit_factory_control: StateInputDisturbanceTrajectoryFactoryInterface = KBSIDTFactory(),
    sit_factory_sim: StateInputDisturbanceTrajectoryFactoryInterface = DBSIDTFactory(),
    vehicle_model_control_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = KinematicBicycle,
    vehicle_model_sim_typename: Type[
        Union[KinematicBicycle, DynamicBicycle, VehicleModelInterface]
    ] = DynamicBicycle,
    state_idxs_typename: Union[KBStateIndices, DBStateIndices] = KBStateIndices,
    input_idxs_typename: Union[KBInputIndices, DBInputIndices] = KBInputIndices,
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
    """
    Combined MPC controller for a planner.
    :param scenario: CommonRoad scenario
    :param planning_problem: CommonRoad planning problem
    :param state_trajectory: planner state trajectory
    :param input_trajectory: planner input trajectory
    :param planner_converter: planner converter
    :param dt_controller: controller time step size in seconds
    :param horizon_ocp: horizon for the optimal control problem in steps
    :param vehicle_params: vehicle parameters object
    :param disturbance_model_typename: typename (=class) of the disturbance
    :param noise_model_typename: typename (=class) of the noise
    :param sit_factory_control: StateInputTrajectory factory for a given dynamics model used in the sim
    :param sit_factory_sim: StateInputTrajectory factory for a given dynamics model used for control
    :param vehicle_model_control_typename: typename (=class) of the vehicle dynamics model used in control
    :param vehicle_model_sim_typename: typename (=class) of the vehicle dynamics model used in sim
    :param state_idxs_typename: typename (=class) of the state idxs, mapping semantic meaning to an idx
    :param input_idxs_typename: typename (=class) of the input idxs, mapping semantic meaning to an idx
    :param sensor_model_typename: typename (=class) of the sensor model / state feedback
    :param func_convert_planner2controller_state: function to convert a planner state into a controller state
    :param func_convert_controller2planner_state: function to convert a controller state into a planner state
    :param ivp_method: IVP-Method used by the ODE-Solver
    :param visualize_scenario: If true, visualizes the scenario
    :param visualize_control: If true, visualizes control and error outputs
    :param save_imgs: If true and img_saving_path is given, saves visualizations instead of displaying them
    :param img_saving_path: If given and save_imgs=true, saves visualizations instead of displaying them
    :return: measured trajectory, trajectory without noise, trajectory without noise and without disturbance
    """
    # TODO: there are some hardcoded params still
    logger.info(f"solving scenario {str(scenario.scenario_id)}")
    x_ref = planner_converter.trajectory_p2c_kb(
        planner_traj=state_trajectory, mode=TrajectoryMode.State
    )
    u_ref = planner_converter.trajectory_p2c_kb(
        planner_traj=input_trajectory, mode=TrajectoryMode.Input
    )

    logger.debug("initialize simulation")
    # ... vehicle model for prediction
    vehicle_model_ctrl = vehicle_model_control_typename(
        params=vehicle_params, delta_t=dt_controller
    )
    # ... initialize optimal control solver
    cost_xx = np.eye(state_idxs_typename.dim)
    cost_xx[state_idxs_typename.steering_angle, state_idxs_typename.steering_angle] = (
        0.0
    )
    cost_uu = 0.1 * np.eye(input_idxs_typename.dim)
    cost_final = cost_xx  # np.eye(KBStateIndices.dim)
    # ... solver parameters for initial step-> iterate until convergence
    solver_parameters_init = SCvxParameters()
    # ... solver parameters for real time iteration -> only one iteration per time step
    solver_parameters_rti = SCvxParameters(max_iterations=1)
    # ... ocp solver (initial parameters)
    scvx_solver = OptimalControlSCvx(
        vehicle_model=vehicle_model_ctrl,
        sit_factory=sit_factory_control,
        horizon=horizon_ocp,
        delta_t=dt_controller,
        cost_xx=cost_xx,
        cost_uu=cost_uu,
        cost_final=cost_final,
        ocp_parameters=solver_parameters_init,
    )
    # instantiate model predictive controller
    mpc = ModelPredictiveControl(ocp_solver=scvx_solver)

    # simulation
    # ... disturbance model
    vehicle_model_sim = vehicle_model_sim_typename(
        params=vehicle_params, delta_t=dt_controller
    )
    sim_disturbance_model: UncertaintyModelInterface = disturbance_model_typename(
        dim=vehicle_model_sim.state_dimension,
        mean=vehicle_params.disturbance_gaussian_mean,
        std_deviation=vehicle_params.disturbance_gaussian_std,
    )
    # ... noise
    sim_noise_model: UncertaintyModelInterface = noise_model_typename(
        dim=vehicle_model_sim.disturbance_dimension,
        mean=vehicle_params.noise_gaussian_mean,
        std_deviation=vehicle_params.noise_gaussian_std,
    )
    # ... sensor model
    sensor_model: SensorModelInterface = sensor_model_typename(
        noise_model=sim_noise_model,
        state_output_factory=sit_factory_sim,
        state_dimension=sit_factory_sim.state_dimension,
        input_dimension=sit_factory_sim.input_dimension,
    )
    # ... simulation
    simulation: Simulation = Simulation(
        vehicle_model=vehicle_model_sim,
        sidt_factory=sit_factory_sim,
        disturbance_model=sim_disturbance_model,
        random_disturbance=True,
        sensor_model=sensor_model,
        random_noise=True,
    )

    # timer
    t_0 = time.perf_counter()
    eta: float = 0.0

    # extend reference trajectory
    clcs_traj, x_ref_ext, u_ref_ext = extend_kb_reference_trajectory_lane_following(
        x_ref=copy.copy(x_ref),
        u_ref=copy.copy(u_ref),
        lanelet_network=scenario.lanelet_network,
        vehicle_params=vehicle_params,
        delta_t=dt_controller,
        horizon=mpc.horizon,
    )
    reference_trajectory = ReferenceTrajectoryFactory(
        delta_t_controller=dt_controller,
        sidt_factory=KBSIDTFactory(),
        mpc_horizon=mpc.horizon,
    )
    # ... dummy reference trajectory: all inputs set to zero
    u_np = np.zeros((u_ref_ext.dim, len(u_ref_ext.steps)))
    u_ref_0 = sit_factory_control.trajectory_from_numpy_array(
        traj_np=u_np,
        mode=TrajectoryMode.Input,
        time_steps=u_ref_ext.steps,
        t_0=u_ref_ext.t_0,
        delta_t=u_ref_ext.delta_t,
    )
    reference_trajectory.set_reference_trajectory(
        state_ref=x_ref_ext, input_ref=u_ref_0, t_0=0
    )

    # simulation results
    x_measured = func_convert_planner2controller_state(
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
            t=kk_sim * dt_controller
        )

        # convert initial state to kb
        x0_kb = func_convert_controller2planner_state(traj_dict_measured[kk_sim])

        # compute control input
        u_now = mpc.compute_control_input(x0=x0_kb, x_ref=tmp_x_ref, u_ref=tmp_u_ref)
        if kk_sim == 0:
            # at the initial step: iterate until convergence
            # afterwards: real-time iteration
            mpc.ocp_solver.reset_ocp_parameters(
                new_ocp_parameters=solver_parameters_rti
            )

        # u_now = kb_input.points[kk_sim]
        input_dict[kk_sim] = u_now
        # simulate
        eta = eta + time.perf_counter() - t_0
        x_measured, x_disturbed, x_nominal = simulation.simulate(
            x0=traj_dict_no_noise[kk_sim],
            u=u_now,
            t_final=dt_controller,
            ivp_method=ivp_method,
        )
        t_0 = time.perf_counter()
        traj_dict_measured[kk_sim + 1] = x_measured
        traj_dict_no_noise[kk_sim + 1] = x_disturbed.final_point

    logger.debug(f"Control took: {eta * 1000} millisec.")
    simulated_traj = sit_factory_sim.trajectory_from_points(
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
            time_steps=list(simulated_traj.points.keys()),
            save_img=save_imgs,
            save_path=img_saving_path,
        )

    return traj_dict_measured, traj_dict_no_noise, input_dict
