import math

from typing import Tuple


def compute_slip_angle_from_steering_angle_in_cog(
    steering_angle: float,
    l_wb: float,
    l_r: float
) -> float:
    """
    Compute slip angle in center of gravity from steering angle and wheelbase.
    :param steering_angle: steering angle
    :param l_wb: wheelbase
    :param l_r: distance from center of rear-axle to the center of gravity
    :return: slip angle in radian
    """
    return math.atan(math.tan(steering_angle) * l_r / l_wb)


def compute_velocity_components_from_slip_angle_and_velocity_in_cog(
        slip_angle: float,
        velocity: float
) -> Tuple[float, float]:
    """
    Compute velocity components in long. and lat. in center of gravity from slip angle.
    :param slip_angle: slip angle
    :param velocity: total velocity
    :return: v_lon, v_lat
    """
    return math.cos(slip_angle) * velocity, math.sin(slip_angle) * velocity


def compute_velocity_components_from_steering_angle_in_cog(
        steering_angle: float,
        velocity_cog: float,
        l_wb: float,
        l_r: float
) -> Tuple[float, float]:
    """
    Computes velocity components at center of gravity. To this end, the slip angle is derived from the steering angle
    using kinematic relations.
    :param steering_angle: steering angle
    :param velocity_cog: velocity at center of gravity
    :param l_wb: wheelbase
    :param l_r: distance from center of rear-axle to the center of gravity
    :return: v_lon, v_lat
    """
    slip_angle: float = compute_slip_angle_from_steering_angle_in_cog(
        steering_angle=steering_angle,
        l_wb=l_wb,
        l_r=l_r
    )

    return compute_velocity_components_from_slip_angle_and_velocity_in_cog(
        slip_angle=slip_angle,
        velocity=velocity_cog
    )


def compute_total_velocity_from_components(
        v_long: float,
        v_lat: float
) -> float:
    """
    Compute length of velocity vector given its components.
    :param v_long: longitudinal velocity
    :param v_lat: lateral velocity
    :return: v
    """
    return math.sqrt(v_long**2 + v_lat**2)


def map_velocity_from_ra_to_cog(
        l_wb: float,
        l_r: float,
        velocity_ra: float,
        steering_angle: float) \
        -> float:
    """
    Given the velocity at the center of the rear axle, this function computes the velocity at the vehicle's center of
    gravity using kinematic relations.
    :param l_wb: wheelbase
    :param l_r: distance from center of rear-axle to the center of gravity
    :param velocity_ra: velocity at the center of the rear axle
    :param steering_angle: steering angle
    :return: velocity at the center of gravity
    """
    if abs(steering_angle) > 1e-6:
        v_ra = velocity_ra
        len_ray_ra = abs(l_wb / math.tan(steering_angle))
        len_ray_cog = math.sqrt(len_ray_ra ** 2 + l_r ** 2)
        v_cog = v_ra * len_ray_cog / len_ray_ra
    else:
        # for steering_angle = 0, the velocities are identical
        v_cog = velocity_ra

    return v_cog

def compute_position_of_cog_from_ra_cc(
        l_r: float,
        position_ra_x: float,
        position_ra_y: float,
        heading: float) \
    -> Tuple[float, float]:
    """
    Given the position of the center of the rear-axle, this function returns the position of the center of gravity; each
    represented in Cartesian coordinates.
    :param l_r: distance from center of rear-axle to the center of gravity
    :param position_ra_x: longitudinal component of the position of the rear-axle (Cartesian coordinates)
    :param position_ra_y: lateral component of the position of the rear-axle (Cartesian coordinates)
    :param heading: orientation of the vehicle
    :return: position of the center of gravity (Cartesian coordinates)
    """
    #TODO: for now just pass rear-axle position
    position_cog_x = position_ra_x# + l_r*math.cos(heading)
    position_cog_y = position_ra_y# + l_r*math.sin(heading)

    return (
        position_cog_x,
        position_cog_y
    )

