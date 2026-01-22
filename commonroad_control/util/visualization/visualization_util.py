from typing import Any, Dict

from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface


def time_synchronize_trajectories(
    reference_trajectory: TrajectoryInterface,
    asynchronous_trajectory: TrajectoryInterface,
    sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface,
) -> TrajectoryInterface:
    """
    Resample an asynchronous trajectory at the sampling times of a reference trajectory.
    :param reference_trajectory: reference trajectory defining sampling times
    :param asynchronous_trajectory: trajectory to be resampled
    :param sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface (required for interpolating states at sampling times)
    :return: synchronized trajectory
    """

    if reference_trajectory.mode != asynchronous_trajectory.mode:
        raise ValueError("Reference and asynchronous trajectories must have the same mode")

    resampled_points: Dict[int, Any] = {}

    for step in reference_trajectory.steps:
        t = reference_trajectory.t_0 + step * reference_trajectory.delta_t

        # Clamp time to valid range of async trajectory, interpolate in between
        if t < asynchronous_trajectory.t_0:
            point = asynchronous_trajectory.initial_point
        elif t > asynchronous_trajectory.t_final:
            point = asynchronous_trajectory.final_point
        else:
            point = asynchronous_trajectory.get_point_at_time(time=t, sidt_factory=sidt_factory)

        resampled_points[step] = point

    return sidt_factory.trajectory_from_points(
        trajectory_dict=resampled_points,
        delta_t=reference_trajectory.delta_t,
        mode=asynchronous_trajectory.mode,
        t_0=reference_trajectory.t_0,
    )
