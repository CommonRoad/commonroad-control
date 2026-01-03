import copy
import logging
from pathlib import Path
from typing import Optional, Union, Callable

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import (
    pid_with_lookahead_for_reactive_planner,
)
from commonroad_control.cr_control_easy_api.mpc_for_dedicated_planner import (
    mpc_for_reactive_planner
)

from commonroad_control.planning_converter.reactive_planner_converter import (
    ReactivePlannerConverter,
)
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import (
    run_reactive_planner,
)
from commonroad_control.util.replanning import (
    si_list_to_si_dict,
    update_planning_problem,
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
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

logger_global = configure_toolbox_logging(level=logging.INFO)
logger = logging.getLogger(__name__)


def main(
    scenario_xml: Union[str, Path],
    planner_config_yaml: Union[str, Path],
    planning_cycle_steps: int,
    max_replanning_iterations: int,
    planner_func: Callable,
    img_save_path: Optional[Path] = None,
    save_imgs: bool = False,
    create_scenario: bool = True,
) -> None:
    """
    Example function for closing the planning-control loop with a PID controller and the reactive planner.
    :param scenario_file: Path to scenario file
    :param scenario_name: Scenario name
    :param img_save_path: Path to save images to
    :param planner_config_path: Reactive planner config
    :param save_imgs: If true and img_save_path is valid, save imgs to this path
    :param create_scenario: Needs to be true for any visualization to happen
    """
    scenario_name = str(scenario_xml).split(".")[0]
    scenario, planning_problem_set = CommonRoadFileReader(scenario_xml).open()
    planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
    planning_problem_original = copy.copy(planning_problem)

    logger.info(f"solving scenario {str(scenario.scenario_id)}")

    rpc = ReactivePlannerConverter()

    overall_planned_traj = list()
    overall_controlled_traj = list()
    replanning_cnt: int = 0
    while replanning_cnt < max_replanning_iterations:
        replanning_cnt += 1

        logger.info(f" -- Replanning cycle {replanning_cnt}")

        # run planner
        logger.debug("run planner")
        rp_states, rp_inputs = run_reactive_planner(
            scenario=scenario,
            scenario_xml_file_name=str(scenario_name + ".xml"),
            planning_problem=planning_problem,
            planning_problem_set=planning_problem_set,
            reactive_planner_config_path=planner_config_yaml,
            evaluate_planner=False,
        )

        if len(rp_states) == 0 or len(rp_inputs) == 0:
            logger.warning(
                f"Planner did not find a trajectory in replanning cycle {replanning_cnt}. "
                f"This can be caused by several planner internal issues e.g. "
                f"- Planner and controller have slightly different definitions on whether the goal is reached"
                f"Aborting!"
            )
            break

        # CommonRoad control expects the trajectories to start at time step (index) zero.
        time_shifted_rp_states = rpc.time_shift_state_input_list(
            si_list=rp_states, t_0=0
        )
        time_shifted_rp_inputs = rpc.time_shift_state_input_list(
            si_list=rp_inputs, t_0=0
        )

        # In this example, we run the reactive planner for the entire planning problem and only take
        # the first steps associated with the planning_cycle step. You can customize your planner to only
        # solve parts of the planning problem in each iteration
        logger.debug("run controller")
        noisy_traj, disturbed_traj, input_traj = (
            planner_func(
                scenario=scenario,
                planning_problem=planning_problem,
                reactive_planner_state_trajectory=time_shifted_rp_states[
                    :planning_cycle_steps
                ],
                reactive_planner_input_trajectory=time_shifted_rp_inputs[
                    :planning_cycle_steps
                ],
                visualize_scenario=False,
                visualize_control=False,
                save_imgs=False,
                img_saving_path=None,
            )
        )

        # Track the planned and driven trajectory
        replanning_step = max(noisy_traj.keys())
        last_state = noisy_traj[replanning_step]
        overall_planned_traj.extend(rp_states[:replanning_step])
        overall_controlled_traj.extend(list(noisy_traj.values())[:replanning_step])

        # Update the planning problem so the planner starts from the current step of the controller
        planning_problem, planning_problem_set = update_planning_problem(
            planning_problem=planning_problem,
            planning_problem_set=planning_problem_set,
            time_step=replanning_step,
            x=last_state.position_x,
            y=last_state.position_y,
            velocity=last_state.velocity,
            orientation_rad=last_state.heading,
            yaw_rate=last_state.yaw_rate,
            acceleration=0.0,
        )
        planning_problem_set._planning_problem_dict[
            planning_problem.planning_problem_id
        ] = planning_problem

        # Check if the driven trajectory has reached the goal state
        simulated_traj = DBSIDTFactory().trajectory_from_points(
            trajectory_dict=si_list_to_si_dict(overall_controlled_traj, t_0=0),
            mode=TrajectoryMode.State,
            t_0=0,
            delta_t=0.1,
        )
        if simulated_traj.check_goal_reached(
            planning_problem=planning_problem, lanelet_network=scenario.lanelet_network
        ):
            logger.info("Controlled vehicle reached goal")
            break

    # Generate the overall trajectories for visualization
    x_ref = rpc.trajectory_p2c_kb(
        planner_traj=rpc.time_shift_state_input_list(
            si_list=overall_planned_traj, t_0=0
        ),
        mode=TrajectoryMode.State,
    )

    simulated_traj = DBSIDTFactory().trajectory_from_points(
        trajectory_dict=si_list_to_si_dict(overall_controlled_traj, t_0=0),
        mode=TrajectoryMode.State,
        t_0=0,
        delta_t=0.1,
    )

    logger.info("Visualization")
    if create_scenario:
        visualize_trajectories(
            scenario=scenario,
            planning_problem=planning_problem_original,
            planner_trajectory=x_ref,
            controller_trajectory=simulated_traj,
            save_path=img_save_path,
            save_img=save_imgs,
        )

        if save_imgs:
            make_gif(
                path_to_img_dir=img_save_path,
                scenario_name=str(scenario.scenario_id),
                num_imgs=len(x_ref.points.values()),
            )

        visualize_reference_vs_actual_states(
            reference_trajectory=x_ref,
            actual_trajectory=simulated_traj,
            time_steps=list(simulated_traj.points.keys())[:-2],
            save_img=save_imgs,
            save_path=img_save_path,
        )


if __name__ == "__main__":
    scenario_name = "ITA_Foggia-6_1_T-1"
    scenario_file = (
        Path(__file__).parents[0] / "scenarios" / str(scenario_name + ".xml")
    )
    planner_config_path = (
        Path(__file__).parents[0]
        / "scenarios"
        / "reactive_planner_configs"
        / str(scenario_name + ".yaml")
    )
    img_save_path = Path(__file__).parents[0] / "output" / scenario_name

    planning_cycle_steps: int = 17
    max_replanning_iterations: int = 6

    # You can hand over other planner functions or write your own
    pid_planner_func = pid_with_lookahead_for_reactive_planner
    mpc_planner_func = mpc_for_reactive_planner

    main(
        scenario_xml=scenario_file,
        planner_config_yaml=planner_config_path,
        planner_func=pid_planner_func,
        planning_cycle_steps=planning_cycle_steps,
        max_replanning_iterations=max_replanning_iterations,
        img_save_path=img_save_path,
        create_scenario=True,
        save_imgs=True,
    )


