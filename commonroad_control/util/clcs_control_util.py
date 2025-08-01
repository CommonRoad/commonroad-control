import numpy as np
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from scipy.spatial.kdtree import KDTree

from commonroad_route_planner.fast_api.fast_api import generate_reference_path_from_lanelet_network_and_planning_problem
from commonroad_route_planner.reference_path import ReferencePath


def extend_ref_path_with_route_planner(
        positional_trajectory: np.ndarray,
        lanelet_network: LaneletNetwork,
        final_state_time_step: int = 0,
        planning_problem_id: int = 30000
) -> np.ndarray:
    """
    Extends the positional information of the trajectory using the CommonRoad route planner.
    :param positional_trajectory: (n,2)
    :param lanelet_network:
    :param planning_problem_id:
    :return:
    """

    initial_state: InitialState = InitialState(
        position=np.asarray(positional_trajectory[-1]),
        orientation=0.0,
        velocity=0.0,
        yaw_rate=0.0,
        acceleration=0.0,
        slip_angle=0.0,
        time_step=final_state_time_step
    )

    goal_region: GoalRegion = GoalRegion(
        state_list=list()
    )

    planning_problem: PlanningProblem = PlanningProblem(
        planning_problem_id=planning_problem_id,
        initial_state=initial_state,
        goal_region=goal_region

    )

    reference_path: ReferencePath = generate_reference_path_from_lanelet_network_and_planning_problem(
        planning_problem=planning_problem,
        lanelet_network=lanelet_network
    )

    kd_tree: KDTree = KDTree(reference_path.reference_path)
    _, idx = kd_tree.query(positional_trajectory[-1])

    clcs_line: np.ndarray = np.concatenate(
        (
            positional_trajectory,
            reference_path.reference_path[min(reference_path.reference_path.shape[0] - 1, idx + 1):]
        )
    )

    return clcs_line



