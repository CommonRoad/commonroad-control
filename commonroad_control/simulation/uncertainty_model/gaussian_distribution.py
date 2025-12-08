import logging
from typing import List, Union

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
)
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)

logger = logging.getLogger(__name__)


class GaussianDistribution(UncertaintyModelInterface):
    """
    Uncertainty model for generating Gaussian noise or disturbances.
    """

    def __init__(
        self,
        dim: int,
        mean: Union[np.ndarray, List[float], UncertaintyInterface],
        std_deviation: Union[np.ndarray, List[float], UncertaintyInterface],
        *args,
        nominal_value: Union[
            np.ndarray, List[float], UncertaintyInterface, None
        ] = None,
        **kwargs,
    ) -> None:
        """
        Initialize uncertainty model. If no user-defined nominal value is provided, the mean of the Gaussian distribution serves as the nominal value.
        :param dim: dimension of the uncertainty - int
        :param mean: mean value - array/ list of floats/ instance of class UncertaintyInterface
        :param std_deviation: standard deviations (component-wise) -  array/ list of floats/ instance of class UncertaintyInterface
        :param nominal_value: if not None: user-defined nominal value of the uncertainty - array/ list of floats/ instance of class UncertaintyInterface
        """
        super().__init__(dim=dim)

        if isinstance(mean, UncertaintyInterface):
            mean_np = mean.convert_to_array()
        else:
            mean_np: np.ndarray = np.array(mean)
        self._mean: np.ndarray = mean_np

        if isinstance(std_deviation, UncertaintyInterface):
            std_deviation_np = std_deviation.convert_to_array()
        else:
            std_deviation_np = np.array(std_deviation)
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
            logger.error("Standard deviation must be non-negative.")
            raise ValueError("Standard deviation must be non-negative.")

    @property
    def mean(self) -> np.ndarray:
        """
        :return: mean value
        """
        return self._mean

    @property
    def std_deviations(self) -> np.ndarray:
        """ "
        :return: standard deviations per entry
        """
        return self._std_deviation

    @property
    def nominal_value(self) -> np.ndarray:
        """
        Returns the nominal value, which is either the user-defined nominal value (passed as an input argument) or the mean of the Gaussian distribution
        :return: np.ndarray of dimension (self.dim,)
        """
        return self._nominal_value

    def sample_uncertainty(self) -> np.ndarray:
        """
        Generates a random sample from the Gaussian distribution.
        :return: np.ndarray of dimension (self.dim,)
        """
        return np.random.normal(self._mean, self._std_deviation, size=self._dim)
