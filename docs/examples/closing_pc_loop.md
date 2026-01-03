# Closing the Planning-Control Loop

An essential part of autonomous driving is the intertwined nature of planning, control and system dynamics.
The controller may not be able to follow the motion plan exactly and real-world uncertainties cause the dynamic system
to not exactly follow the controller trajectory.

Real-world systems require frequent closed-loop replanning from the actual current system state for uncertainties.
This example shows how to close the loop between a motion planner, the controller aiming to follow that motion
plan and the system dynamics (simulation).

## Preliminaries

### Choosing your planner and controller

A smooth interaction between planner, controller and system is highly dependent on the individual
setup. Thus, we recommend to carefully choose the planner and controller. We recommend choosing a simple
planner and controller first, then upgrade the planner next.

In our experience, many research-grade motion planners suffer severe degradation of performance
when executed in the closed-loop. We attribute this, amongst other problems, to too strict requirements for the initial state.

### Collision Checks for Control

In many real-world autonomous driving stacks, control does not have the opportunity to perform additional
collision checks. You can integrate additional collision checks by using the **CommonRoad Collision Checker**, although
a conversion of the trajectory states may be required.

### Harmonizing Time Conventions between Planning and Control

Control usually assumes that the trajectory always starts at time step zero, which can
correspond to an arbitrary time. Motion planners typically require their time-steps to be
synchronized to the overall scenario time steps, for example to perform collision checks with dynamic
obstacles. Thus, in each replanning cycle after the first, the planner requires consecutive time steps
whereas control starts at time step zero again.

When closing the loop between planning and control, one must harmonize the time step conventions
between planning and control. In this example, we time-shift the planner states and inputs
in each replanning cycle.

### Harmonizing Goal Checks between Planning, Control and the Real System

Motion planners are typically able to check whether a state is within the goal region. Due to
the difference between the planned trajectory, the controller trajectory and the actual system
trajectory, a planner may plan a trajectory that should reach the goal but the real system may not
entirely be able to reach it.

Thus, additional goal checks for the actual (simulated) trajectory are necessary. Depending
on your individual project, it may be useful to slightly inflate the goal region for the system
goal check to avoid additional planning cycles when the system barely does not reach the goal due
to planner/controller/system differences.

## Setup

We use the **CommonRoad Reactive Planner** and our [**Long-Lat separated PID Controller**](../core_api/control/pid.md).
To make the example easier to understand, we let the Reactive planner always plan to the position instead
of a fixed time horizon, although this is easily integratable.

As many Frenet-based planners, the Reactive planner is very sensitive to its state with respect to
the reference path. Thus, the (experimentally found) choice of the replanning cycle step for each scenario is important for a stable loop.

## Swapping PID and MPC
The example below uses our Long-Lat PID controller. You can swap 
```Python3
...
pid_with_lookahead_for_reactive_planner( ... )
...
```

with 

```Python3
...
mpc_for_reactive_planner( ... )
...
```
to use our MPC


## Overall Example
```Python3
import copy
import logging
from pathlib import Path
from typing import Optional, Union

from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import (
    pid_with_lookahead_for_reactive_planner,
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
            pid_with_lookahead_for_reactive_planner(
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
    # You may have to modify the paths 
    scenario_name = "ZAM_Tutorial_Urban-3_2"
    scenario_file = "PATH/TO/SCENARIO.xml"
    planner_config_path = "PATH/TO/REACTIVEPLANNER/CONFIG.yaml"
    img_save_path = "PATH/TO/SAVE/IMGS/TO"

    # You may have to adjust these values depending on your planner
    planning_cycle_steps: int = 10
    max_replanning_iterations: int = 6

    main(
        scenario_xml=scenario_file,
        planner_config_yaml=planner_config_path,
        planning_cycle_steps=planning_cycle_steps,
        max_replanning_iterations=max_replanning_iterations,
        img_save_path=img_save_path,
        create_scenario=True,
        save_imgs=True,
    )
```
