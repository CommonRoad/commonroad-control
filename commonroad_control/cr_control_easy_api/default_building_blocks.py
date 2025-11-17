from commonroad_control.simulation.uncertainty_model.gaussian_distribution import GaussianDistribution

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBStateIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBStateIndices


def gaussian_disturbance_for_db() -> GaussianDistribution:
    """
    :return: Gaussian disturbances of the dynamic bicycle model.
    """
    return GaussianDistribution(
        dim=DBStateIndices.dim,
        mean=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviation=[0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_noise_for_db() -> GaussianDistribution:
    """
    :return: Gaussian (measurement) noise for simulating the dynamic bicycle model.
    """
    return GaussianDistribution(
        dim=DBStateIndices.dim,
        mean=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviation=[0.075, 0.075, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_disturbance_for_kb() -> GaussianDistribution:
    """
    :return: Gaussian disturbances of the kinematic bicycle model.
    """
    return GaussianDistribution(
        dim=KBStateIndices.dim,
        mean=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviation=[0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_noise_for_kb() -> GaussianDistribution:
    """
    :return: Gaussian (measurement) noise for simulating the kinematic bicycle model.
    """
    return GaussianDistribution(
        dim=KBStateIndices.dim,
        mean=[0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviation=[0.075, 0.075, 0.0, 0.0, 0.0]
    )


















