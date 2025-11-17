from typing import List, Optional, Union
import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface


class GaussianDistribution(UncertaintyModelInterface):
    """
    Generates Gaussian noise or disturbances
    """

    def __init__(
            self,
            dim: int,
            mean: Union[np.ndarray,List[float]],
            std_deviation: Union[np.ndarray,List[float]],
            nominal_value: Optional[List[float]] = None
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

        self._mean: np.ndarray = np.array(mean)
        self._std_deviation: np.ndarray = np.array(std_deviation)

        # set nominal value
        if nominal_value is not None:
            self._nominal_value = np.array(nominal_value)
        else:
            self._nominal_value = self._mean

        self._sanity_check()

    def _sanity_check(self) -> None:
        """
        Checks args
        """
        if len(self._mean) != len(self._std_deviation) != self._dim:
            raise ValueError(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, mean:{len(self._mean)}, std:{len(self._std_deviation)}"
            )
        if any(self._std_deviation < 0):
            raise ValueError(
                f"Standard deviation must be non-negative."
            )

    @property
    def mean(self) -> np.ndarray:
        """
        :return: mean value
        """
        return self._mean

    @property
    def std_deviations(self) -> np.ndarray:
        """"
        :return: standard deviations per entry
        """
        return self._std_deviation

    @property
    def nominal_value(self) -> np.ndarray:
        """
        :return: nominal value of the uncertainty
        """
        return self._nominal_value

    def sample_uncertainty(self) -> np.ndarray:
        """
        Generates a random sample from the Gaussian distribution.
        :return:
        """
        return np.random.normal(self._mean, self._std_deviation, size=self._dim)
