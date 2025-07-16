from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState

def convert_state_kst2dst(kst_state: "KSTState") -> DBState:
    return DBState(
        position_x=kst_state.position_x,
        position_y=kst_state.position_y,
        velocity_long=kst_state.velocity,
        velocity_lat=0.0,
        heading=kst_state.heading,
        yaw_rate=0.0,
        steering_angle=kst_state.steering_angle
    )