from math import cos, sin

from commonroad_control.util.conversion_util import (
    compute_slip_angle_from_steering_angle_in_cog,
    compute_velocity_from_components,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


def convert_state_kb2db(kb_state: "KBState", vehicle_params: VehicleParameters) -> DBState:
    """
    Converts kinematic bicycle state to dynamic bicycle state.
    :param kb_state: KB state
    :param vehicle_params: Vehicle parameters
    :return: KB state
    """

    slip_angle = compute_slip_angle_from_steering_angle_in_cog(
        steering_angle=kb_state.steering_angle,
        velocity=kb_state.velocity,
        l_wb=vehicle_params.l_wb,
        l_r=vehicle_params.l_r,
    )

    return DBState(
        position_x=kb_state.position_x,
        position_y=kb_state.position_y,
        velocity_long=kb_state.velocity * cos(slip_angle),
        velocity_lat=kb_state.velocity * sin(slip_angle),
        heading=kb_state.heading,
        yaw_rate=kb_state.velocity * sin(slip_angle) / vehicle_params.l_r,
        steering_angle=kb_state.steering_angle,
    )


def convert_state_db2kb(db_state: "DBState") -> KBState:
    """
    Converts dynamic bicycle state to kinematic bicycle state.
    :param db_state: DB state
    :return: KB State
    """
    v = compute_velocity_from_components(v_long=db_state.velocity_long, v_lat=db_state.velocity_lat)

    return KBState(
        position_x=db_state.position_x,
        position_y=db_state.position_y,
        velocity=v,
        heading=db_state.heading,
        steering_angle=db_state.steering_angle,
    )
