# standard imports
import logging
from pathlib import Path
from typing import List, Tuple, Union

# commonroad
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.state import InputState

# reactive planner
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.state import ReactivePlannerState
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad_rp.utility.evaluation import run_evaluation
from commonroad_rp.utility.logger import initialize_logger
from commonroad_rp.utility.utils_coordinate_system import (
    CoordinateSystem,
    create_coordinate_system,
    create_initial_ref_path,
)

logger = logging.getLogger(__name__)


def run_reactive_planner(
    scenario: Scenario,
    scenario_xml_file_name: str,
    planning_problem: PlanningProblem,
    planning_problem_set: PlanningProblemSet,
    reactive_planner_config_path: Union[str, Path],
    logging_level: str = "ERROR",
    show_planner_debug_plots: bool = False,
    maximum_iterations: int = 200,
) -> Tuple[List[ReactivePlannerState], List[InputState]]:
    """
    Util wrapper to easily run the reactive planner
    :param scenario: CommonRoad scenario object
    :param scenario_xml_file_name: e.g. ZAM-1_1.xml
    :param planning_problem: CommonRoad planning problem object
    :param planning_problem_set: planning problem set orbject
    :param reactive_planner_config_path: path to reactive planner config
    :param logging_level: logging level as string (see reactive planner documentation)
    :param show_planner_debug_plots: if true shows debug plots
    :param maximum_iterations: maximum planner iterations to solve a problem before raising an exception
    :return: Tuple[list of reactive planner states, list of reactive planner inputs]
    """

    config = ReactivePlannerConfiguration.load(
        reactive_planner_config_path, scenario_xml_file_name
    )
    config.update(scenario=scenario, planning_problem=planning_problem)
    config.planning_problem_set = planning_problem_set
    config.debug.logging_level = logging_level
    config.debug.show_plots = show_planner_debug_plots

    # initialize and get logger
    rp_logger = initialize_logger(config)
    rp_logger.setLevel(level=logger.getEffectiveLevel())

    ref_path_orig = create_initial_ref_path(
        config.scenario.lanelet_network, config.planning_problem
    )
    rp_cosys: CoordinateSystem = create_coordinate_system(ref_path_orig)

    # initialize reactive planner
    planner = ReactivePlanner(config)
    planner.set_reference_path(coordinate_system=rp_cosys)

    # Run Planning
    planner.record_state_and_input(planner.x_0)

    SAMPLING_ITERATION_IN_PLANNER = True

    cnt: int = 0

    while not planner.goal_reached() and cnt < maximum_iterations:
        current_count = len(planner.record_state_list) - 1

        # check if planning cycle or not
        plan_new_trajectory = current_count % config.planning.replanning_frequency == 0
        if plan_new_trajectory:
            # new planning cycle -> plan a new optimal trajectory
            planner.set_desired_velocity(current_speed=planner.x_0.velocity)
            if SAMPLING_ITERATION_IN_PLANNER:
                optimal = planner.plan()
            else:
                optimal = None
                i = 1
                while optimal is None and i <= planner.sampling_level:
                    optimal = planner.plan(i)
                    i += 1

            if not optimal:
                break

            # record state and input
            planner.record_state_and_input(optimal[0].state_list[1])

            # reset planner state for re-planning
            planner.reset(
                initial_state_cart=planner.record_state_list[-1],
                initial_state_curv=(optimal[1][1], optimal[2][1]),
                collision_checker=planner.collision_checker,
                coordinate_system=planner.coordinate_system,
            )

        else:
            # continue on optimal trajectory
            temp = current_count % config.planning.replanning_frequency

            # record state and input
            planner.record_state_and_input(optimal[0].state_list[1 + temp])

            # reset planner state for re-planning
            planner.reset(
                initial_state_cart=planner.record_state_list[-1],
                initial_state_curv=(optimal[1][1 + temp], optimal[2][1 + temp]),
                collision_checker=planner.collision_checker,
                coordinate_system=planner.coordinate_system,
            )

    if cnt >= maximum_iterations - 1:
        logger.error(
            f"Reactive planner exceeded maximum number of iterations {maximum_iterations}"
        )
        raise Exception(
            f"Reactive planner exceeded maximum number of iterations {maximum_iterations}"
        )

    # Evaluate results
    evaluate = True
    if evaluate:
        _, _ = run_evaluation(
            planner.config, planner.record_state_list, planner.record_input_list
        )

    # Move input up one time step so that the idx of the input correspond the state to which it is applied to to come
    # into the next state
    input_list: List[InputState] = planner.record_input_list[1:]
    for el in input_list:
        el.time_step = el.time_step - 1

    return planner.record_state_list, input_list
