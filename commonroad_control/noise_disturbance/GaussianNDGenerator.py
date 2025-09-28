import numpy as np

from commonroad_control.noise_disturbance.NoiseDisturbanceGeneratorInterface import NoiseDisturbanceGeneratorInterface
from typing import List, Optional


class GaussianNDGenerator(NoiseDisturbanceGeneratorInterface):
    """
    Generates Gaussian noise or disturbances
    """

    def __init__(
            self,
            dim: int,
            means: List[float],
            std_deviations: List[float],
    ) -> None:
        """
        Generates Gaussian noise or disturbances.

        Reasonable Values for disturbance
        std deviation of position (x,y) are 0.001m to 0.025m, given
        a maximum continuous-time diffusion with a drift over 10m between 0.1m and 0.5m and a control time step of
        dt=0.01. For a control time step of dt=0.1, 0.02 to 0.05 is common.
        std deviation of steering angle: 0.0010rad to 0.0017 for control dt=0.01; and 0.0031 to 0.01 for control dt=0.1

        Reasonable values for noise:
        std deviation of position (x,y): Automative Grade GPS+SBAS 0.5m to 1.0m, GNSS+RTK 0.02m – 0.1 m,
        Lidar SLAM 0.01m – 0.05m,
        std deviation for steering angle: 0.0035–0.0087 rad


        :param dim: dimension of vector
        :param mean: list of mean values per entry
        :param std_deviation: list of standard deviations per entry
        """
        super().__init__(dim=dim)
        self._means: List[float] = means
        self._std_deviations: List[float] = std_deviations
        self._sanity_check()


    @property
    def means(self) -> List[float]:
        """
        :return: mean values per entry
        """
        return self._means

    @property
    def std_deviations(self) -> List[float]:
        """"
        :return: standard deviations per entry
        """
        return self._std_deviations

    def apply(
            self,
            state: np.ndarray,
            *args
    ) -> np.ndarray:
        """
        Generate array of Gaussian noise or disturbances
        :param state: state np array (dim,)
        :return: state with added noise or disturbance
        """
        return state + np.random.normal(self._means, self._std_deviations, size=self._dim)


    def _sanity_check(self) -> None:
        """
        Checks args
        """
        if len(self._means) != len(self._std_deviations) != self._dim:
            raise ValueError(
                f"dim, means, std_deviations must have the same length, "
                f"but have dim:{self._dim}, mean:{len(self._means)}, std:{len(self._std_deviations)}"
            )









