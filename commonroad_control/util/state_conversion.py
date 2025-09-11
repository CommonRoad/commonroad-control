from math import sqrt, atan, tan, sin, cos

from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState


def convert_state_kst2dst(
        kst_state: "KSTState",
        vehicle_params: VehicleParameters) \
        -> DBState:
    slip_angle = atan(tan(kst_state.steering_angle) * vehicle_params.l_r / vehicle_params.l_wb)
    return DBState(
        position_x=kst_state.position_x,
        position_y=kst_state.position_y,
        velocity_long=kst_state.velocity*cos(slip_angle),
        velocity_lat=kst_state.velocity*sin(slip_angle),
        heading=kst_state.heading,
        yaw_rate=kst_state.velocity*sin(slip_angle)/vehicle_params.l_r,
        steering_angle=kst_state.steering_angle
    )

def convert_state_dst2kst(
        db_state: "DBState") \
        -> KSTState:
    return KSTState(
        position_x=db_state.position_x,
        position_y=db_state.position_y,
        velocity=sqrt(db_state.velocity_long**2 + db_state.velocity_lat**2),
        heading=db_state.heading,
        steering_angle=db_state.steering_angle
    )