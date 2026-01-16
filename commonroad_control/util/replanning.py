import copy
from typing import Any, Dict, List, Tuple

import numpy as np
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from shapely.geometry.point import Point


def update_planning_problem(
    planning_problem: PlanningProblem,
    planning_problem_set: PlanningProblemSet,
    time_step: int,
    x: float,
    y: float,
    orientation_rad: float,
    velocity: float,
    acceleration: float,
    yaw_rate: float = 0.0,
    slip_angle: float = 0.0,
) -> Tuple[PlanningProblem, PlanningProblemSet]:
    """
    Update commonroad planning problem with new state
    :param planning_problem: Old planning problem
    :param planning_problem_set: old planning problem set
    :param time_step: time step of initial state
    :param x: cartesian x position
    :param y: cartesian y position
    :param orientation_rad: orientation in rad
    :param velocity: velocity
    :param acceleration: acceleration
    :param yaw_rate: optional yaw_rate
    :param slip_angle: optional slip_angle
    :return: New planning problem instance, new planning problem set instance
    """
    initial_state: InitialState = InitialState(
        position=np.asarray([x, y]),
        orientation=orientation_rad,
        velocity=velocity,
        acceleration=acceleration,
        yaw_rate=yaw_rate,
        slip_angle=slip_angle,
        time_step=time_step,
    )
    pp = PlanningProblem(
        planning_problem_id=planning_problem.planning_problem_id,
        initial_state=initial_state,
        goal_region=planning_problem.goal,
    )

    # Workaround for bug in commonroad-io
    pps = copy.copy(planning_problem_set)
    pps._planning_problem_dict[planning_problem.planning_problem_id] = planning_problem

    return pp, pps


def check_position_in_goal_region(
    goal_region: GoalRegion,
    lanelet_network: LaneletNetwork,
    position_x: float,
    position_y: float,
    goal_region_buffer: float = 1.0,
) -> bool:
    """
    Checks geometrically, if position is in goal region using a buffer.
    If the goal does not contain a positional value, it is always false.
    :param goal_region: CommonRoad goal region
    :param lanelet_network: CommonRoad lanelet network
    :param position_x: x position of state
    :param position_y: y position of state
    :param goal_region_buffer: buffer for goal region to counter slight differences between planned and controlled trajectory
    :return: True, if point is in goal region
    """
    is_position_in_goal: bool = False

    if goal_region is not None:
        if goal_region.lanelets_of_goal_position is not None:
            if len(list(goal_region.lanelets_of_goal_position.values())) > 0:
                # check against lanelet geometry
                lanelet_ids = list(goal_region.lanelets_of_goal_position.values())
                for pp_list in lanelet_ids:
                    for lanelet_id in pp_list:
                        lanelet = lanelet_network.find_lanelet_by_id(lanelet_id)
                        is_in_goal = lanelet.polygon.shapely_object.buffer(
                            goal_region_buffer
                        ).contains(Point([position_x, position_y]))
                        if is_in_goal:
                            is_position_in_goal = True
                            break

        else:
            # use state list
            for goal_state in goal_region.state_list:
                if hasattr(goal_state, "position"):
                    if hasattr(goal_state.position, "shapely_object"):
                        if goal_state.position.shapely_object is not None:
                            is_in_goal = goal_state.position.shapely_object.buffer(
                                goal_region_buffer
                            ).contains(Point([position_x, position_y]))
                            if is_in_goal:
                                is_position_in_goal = True
                                break

                    elif hasattr(goal_state.position, "center"):
                        # use position with buffer directly
                        if (
                            np.linalg.norm(
                                np.asarray(goal_state.position.center)
                                - np.asarray([position_x, position_y])
                            )
                            < goal_region_buffer
                        ):
                            is_position_in_goal = True
                            break
                    else:
                        raise NotImplementedError("Goal state description not used")

    return is_position_in_goal


def si_list_to_si_dict(si_list: List[Any], t_0: int) -> Dict[int, Any]:
    """
    Converts a list of states in a dict with the key being the previous list index plus an offset t_0
    :param si_list: state of input list
    :param t_0: time offset
    :return: Dict[time_step, value]
    """
    return {i + t_0: v for i, v in enumerate(si_list)}
