import math

from typing import Tuple, TYPE_CHECKING



def compute_splip_angle_from_steering_angle_in_cog(
    steering_angle: float,
    wheelbase: float,
    length_rear: float
) -> float:
    """
    Compute slip angle in center of gravity from steering angle and wheelbase
    :param steering_angle:
    :param wheelbase:
    :param length_rear:
    :return: slip angle in radian
    """
    return math.atan(math.tan(steering_angle) * length_rear / wheelbase)



def compute_velocity_components_from_slip_angle_and_velocity_in_cog(
        slip_angle: float,
        velocity: float
) -> Tuple[float, float]:
    """
    Compute velocity components in long. and lat. in center of gravity from slip angle
    :param slip_angle:
    :param velocity:
    :return: v_lon, v_lat
    """
    return math.cos(slip_angle) * velocity, math.sin(slip_angle) * velocity


def compute_velocity_components_from_steering_angle_in_cog(
        steering_angle: float,
        velocity: float,
        wheelbase: float,
        length_rear: float
) -> Tuple[float, float]:
    """
    Comptues velocity components in center of gravity from steering angle
    :param steering_angle:
    :param velocity:
    :param wheelbase:
    :param length_rear:
    :return: v_lon, v_lat
    """
    slip_angle: float = compute_splip_angle_from_steering_angle_in_cog(
        steering_angle=steering_angle,
        wheelbase=wheelbase,
        length_rear=length_rear
    )

    return compute_velocity_components_from_slip_angle_and_velocity_in_cog(
        slip_angle=slip_angle,
        velocity=velocity
    )


def compute_total_velocity_from_components(
        v_long: float,
        v_lat: float
) -> float:
    """
    Compute length of velocity vector given its components
    :param v_long:
    :param v_lat:
    :return: v
    """
    return math.sqrt(v_long**2 + v_lat**2)

