from typing import List, Union
import numpy as np
import logging

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.simulation.uncertainty_model.uncertainty_interface import UncertaintyInterface


logger = logging.getLogger(__name__)

class GaussianDistribution(UncertaintyModelInterface):
    """
    Generates Gaussian noise or disturbances
    """

    def __init__(
            self,
            dim: int,
            mean: Union[np.ndarray, List[float], UncertaintyInterface],
            std_deviation: Union[np.ndarray, List[float], UncertaintyInterface],
            *args,
            nominal_value: Union[np.ndarray, List[float], UncertaintyInterface, None] = None,
            **kwargs
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

        if isinstance(mean, UncertaintyInterface):
            mean_np = mean.convert_to_array()
        else:
            mean_np : np.ndarray = np.array(mean)
        self._mean: np.ndarray = mean_np

        if isinstance(std_deviation, UncertaintyInterface):
            std_deviation_np = std_deviation.convert_to_array()
        else:
            std_deviation_np = np.array(mean)
        self._std_deviation: np.ndarray = std_deviation_np

        # set nominal value
        if nominal_value is not None:
            if isinstance(nominal_value, UncertaintyInterface):
                nominal_value_np = nominal_value.convert_to_array()
            else:
                nominal_value_np: np.ndarray = np.array(nominal_value)
            self._nominal_value = nominal_value_np
        else:
            self._nominal_value = self._mean

        self._sanity_check()

    def _sanity_check(self) -> None:
        """
        Checks args
        """
        if len(self._mean) != len(self._std_deviation) != self._dim:
            logger.error(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, mean:{len(self._mean)}, std:{len(self._std_deviation)}"
            )
            raise ValueError(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, mean:{len(self._mean)}, std:{len(self._std_deviation)}"
            )
        if any(self._std_deviation < 0):
            logger.error(f"Standard deviation must be non-negative.")
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
