from commonroad_control.noise_disturbance.GaussianNDGenerator import GaussianNDGenerator


def gaussian_disturbance_for_db() -> GaussianNDGenerator:
    """
    :return: Gaussian disturbance generator parametrized for realistic dynamic bicycle.
    """
    return GaussianNDGenerator(
        dim=7,
        means=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_noise_for_db() -> GaussianNDGenerator:
    """
    :return: Gaussian noise generator parametrized for realistic dynamic bicycle.
    """
    return GaussianNDGenerator(
        dim=7,
        means=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.075, 0.075, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_disturbance_for_kb() -> GaussianNDGenerator:
    """
    :return: Gaussian disturbance generator parametrized for realistic kinematic bicycle.
    """
    return GaussianNDGenerator(
        dim=5,
        means=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0]
    )


def gaussian_noise_for_kb() -> GaussianNDGenerator:
    """
    :return: Gaussian noise generator parametrized for realistic kinematic bicycle.
    """
    return GaussianNDGenerator(
        dim=5,
        means=[0.0, 0.0, 0.0, 0.0, 0.0],
        std_deviations=[0.075, 0.075, 0.0, 0.0, 0.0]
    )


















