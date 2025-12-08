import logging
from math import atan2
from typing import Any, List, Tuple, Union

import numpy as np
from commonroad.planning.goal import GoalRegion
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.scenario.lanelet import LaneletNetwork
from commonroad.scenario.state import InitialState
from commonroad_clcs.clcs import CurvilinearCoordinateSystem
from commonroad_clcs.config import CLCSParams
from commonroad_route_planner.fast_api.fast_api import (
    generate_reference_path_from_lanelet_network_and_planning_problem,
)
from commonroad_route_planner.reference_path import ReferencePath
from scipy.spatial.kdtree import KDTree

from commonroad_control.util.conversion_util import unwrap_angle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters

logger = logging.getLogger(__name__)


def extend_ref_path_with_route_planner(
    positional_trajectory: np.ndarray,
    lanelet_network: LaneletNetwork,
    final_state_time_step: int = 0,
    planning_problem_id: int = 30000,
) -> np.ndarray:
    """
    Extends the positional information of the trajectory using the CommonRoad route planner.
    :param positional_trajectory: (n,2) positional trajectory
    :param lanelet_network: CommonRoad lanelet network object.
    :param planning_problem_id: Id of current planning problem
    :return: (n,2) positional trajectory of extended reference path
    """

    initial_state: InitialState = InitialState(
        position=np.asarray(positional_trajectory[-1]),
        orientation=0.0,
        velocity=0.0,
        yaw_rate=0.0,
        acceleration=0.0,
        slip_angle=0.0,
        time_step=final_state_time_step,
    )

    goal_region: GoalRegion = GoalRegion(state_list=list())

    planning_problem: PlanningProblem = PlanningProblem(
        planning_problem_id=planning_problem_id,
        initial_state=initial_state,
        goal_region=goal_region,
    )

    reference_path: ReferencePath = (
        generate_reference_path_from_lanelet_network_and_planning_problem(
            planning_problem=planning_problem, lanelet_network=lanelet_network
        )
    )

    kd_tree: KDTree = KDTree(reference_path.reference_path)
    _, idx = kd_tree.query(positional_trajectory[-1])

    clcs_line: np.ndarray = np.concatenate(
        (
            positional_trajectory,
            reference_path.reference_path[
                min(reference_path.reference_path.shape[0] - 1, idx + 1) :
            ],
        )
    )

    return clcs_line


def extend_reference_trajectory_lane_following(
    positional_trajectory: np.ndarray,
    lanelet_network: LaneletNetwork,
    final_state: Union[StateInterface, KBState, DBState, Any],
    horizon: int,
    delta_t: float,
    l_wb: float,
) -> Tuple[
    CurvilinearCoordinateSystem,
    List[np.ndarray],
    List[np.ndarray],
    List[float],
    List[float],
    List[float],
    List[float],
]:
    """
    Extends planned trajectory using the CommonRoad reference path planner to account for
    extended MPC horizons, controller overshoot etc.
    :param positional_trajectory: (n,2) positional planner trajectory
    :param lanelet_network: CommonRoad lanelet network
    :param final_state: Final planner trajectory state
    :param horizon: time horizon in steps
    :param delta_t: time step size in seconds
    :param l_wb: wheelbase length
    :return: Curvilinear coordinate system object, (n,2) positions in cartesian coordinates, acceleration, heading, yaw_rate, steering angle, steering angle velocity,
    """
    # extend path with route planner
    clcs_line = extend_ref_path_with_route_planner(
        positional_trajectory, lanelet_network
    )

    # convert to curvilinear coordinate system
    clcs_traj_ext: CurvilinearCoordinateSystem = CurvilinearCoordinateSystem(
        reference_path=clcs_line, params=CLCSParams()
    )

    # sample states along path using velocity of final state
    v_ref = final_state.velocity
    position_s_0, _ = clcs_traj_ext.convert_to_curvilinear_coords(
        x=final_state.position_x, y=final_state.position_y
    )
    position_xy = []
    velocity = [v_ref for _ in range(horizon)]
    heading = []
    yaw_rate = []
    steering_angle = []
    acceleration = [0.0 for _ in range(horizon)]
    steering_angle_velocity = []
    if hasattr(final_state, "heading"):
        heading_0 = final_state.heading
    elif hasattr(final_state, "velocity_y") and hasattr(final_state, "velocity_x"):
        heading_0 = atan2(final_state.velocity_y, final_state.velocity_x)
    else:
        logger.error(
            "Could not compute heading at the final state of the reference trajectory!"
        )
        raise Exception(
            "Could not compute heading at the final state of the reference trajectory!"
        )

    if hasattr(final_state, "steering_angle"):
        steering_angle_0 = final_state.steering_angle
    else:
        steering_angle_0: float = 0.0

    for kk in range(horizon):
        position_s = position_s_0 + (kk + 1) * delta_t * v_ref

        # convert position to Cartesian coordinates
        position_xy.append(
            clcs_traj_ext.convert_to_cartesian_coords(s=position_s, l=0.0)
        )

        # orientation of reference trajectory at
        tangent = clcs_traj_ext.tangent(position_s)
        # ... unwrap heading to ensure continuity (avoid discontinuity)
        tmp_heading = atan2(tangent[1], tangent[0])
        heading.append(unwrap_angle(alpha_prev=heading_0, alpha_next=tmp_heading))

        # compute yaw rate
        yaw_rate.append(float((heading[kk] - heading_0) / delta_t))
        heading_0: float = heading[kk]

        # compute steering angle
        steering_angle.append(float(atan2(yaw_rate[kk] * l_wb, v_ref)))

        # compute steering angle velocity
        steering_angle_velocity.append(
            float((steering_angle[kk] - steering_angle_0) / delta_t)
        )
        steering_angle_0 = steering_angle[kk]

    return (
        clcs_traj_ext,
        position_xy,
        velocity,
        acceleration,
        heading,
        yaw_rate,
        steering_angle,
        steering_angle_velocity,
    )


def extend_kb_reference_trajectory_lane_following(
    x_ref: TrajectoryInterface,
    u_ref: TrajectoryInterface,
    lanelet_network: LaneletNetwork,
    vehicle_params: VehicleParameters,
    delta_t: float,
    horizon: int,
) -> Tuple[CurvilinearCoordinateSystem, TrajectoryInterface, TrajectoryInterface]:
    """
    Extends kinematic bicycle trajectory using the CommonRoad reference path planner to account for
    extended MPC horizons, or controllers with look-ahead etc.
    :param x_ref: State trajectory
    :param u_ref: Input trajectory
    :param lanelet_network: CommonRoad lanelet network
    :param vehicle_params: Vehicle parameters object
    :param delta_t: sampling time in seconds - float
    :param horizon: time horizon for extension (number of time steps) - int
    :return: Curvilinear coordinate system, extended state trajectory, extended input trajectory
    """
    # positional trajectory
    positional_trajectory = np.asarray(
        [[state.position_x, state.position_y] for state in x_ref.points.values()]
    )

    # extend trajectory
    (
        clcs_traj_ext,
        position_xy,
        velocity,
        acceleration,
        heading,
        yaw_rate,
        steering_angle,
        steering_angle_velocity,
    ) = extend_reference_trajectory_lane_following(
        positional_trajectory=positional_trajectory,
        lanelet_network=lanelet_network,
        final_state=x_ref.final_point,
        horizon=horizon,
        delta_t=delta_t,
        l_wb=vehicle_params.l_wb,
    )

    # append states
    for kk in range(horizon):
        x_ref.append_point(
            KBState(
                position_x=position_xy[kk][0],
                position_y=position_xy[kk][1],
                velocity=velocity[kk],
                heading=heading[kk],
                steering_angle=steering_angle[kk],
            )
        )

    # append control inputs
    for kk in range(horizon):
        u_ref.append_point(
            KBInput(
                acceleration=acceleration[kk],
                steering_angle_velocity=steering_angle_velocity[kk],
            )
        )

    return clcs_traj_ext, x_ref, u_ref
