import logging
from typing import Union

import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import (
    UncertaintyInterface,
)
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)

logger = logging.getLogger(__name__)


class UniformDistribution(UncertaintyModelInterface):
    """
    Uncertainty model for generating uniformly distributed noise or disturbances.
    """

    def __init__(
        self,
        dim: int,
        lower_bound: Union[np.ndarray, list[float], UncertaintyInterface],
        upper_bound: Union[np.ndarray, list[float], UncertaintyInterface],
        *args,
        nominal_value: Union[np.ndarray, list[float], UncertaintyInterface] = None,
        **kwargs,
    ) -> None:
        """
        Initialize uncertainty model. If no user-defined nominal value is provided, the mean of the uniform distribution serves as the nominal value.
        :param dim: dimension of the uncertainty - int
        :param lower_bound: lower bound of the uncertainty values - array/ list of floats/ instance of class UncertaintyInterface
        :param upper_bound: upper bound of the uncertainty values - array/ list of floats/ instance of class UncertaintyInterface
        :param nominal_value: if not None: user-defined nominal value of the uncertainty - array/ list of floats/ instance of class UncertaintyInterface
        """

        super().__init__(dim=dim)

        if isinstance(lower_bound, UncertaintyInterface):
            lower_bound_np = lower_bound.convert_to_array()
        else:
            lower_bound_np: np.ndarray = np.array(lower_bound)
        self._lower_bound: np.ndarray = lower_bound_np

        if isinstance(upper_bound, UncertaintyInterface):
            upper_bound_np = upper_bound.convert_to_array()
        else:
            upper_bound_np: np.ndarray = np.array(upper_bound)
        self._upper_bound: np.ndarray = upper_bound_np

        # set nominal value
        if nominal_value is not None:
            if isinstance(nominal_value, UncertaintyInterface):
                nominal_value_np = nominal_value.convert_to_array()
            else:
                nominal_value_np: np.ndarray = np.array(nominal_value)
            self._nominal_value: np.ndarray = nominal_value_np
        else:
            self._nominal_value = 0.5 * (self._upper_bound + self._lower_bound)

        self._sanity_check()

    def _sanity_check(self) -> None:
        """
        Check args.
        """
        # check dimension
        if len(self._lower_bound) != self._dim or len(self._upper_bound) != self._dim:
            logger.error(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, "
                f"lower bound:{len(self._lower_bound)}, upper bound:{len(self._upper_bound)}"
            )
            raise ValueError(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, "
                f"lower bound:{len(self._lower_bound)}, upper bound:{len(self._upper_bound)}"
            )
        # check bounds (lb <= ub)
        if any(self._upper_bound < self._lower_bound):
            logger.error("Upper bound must be greater than lower bound.")
            raise ValueError("Upper bound must be greater than lower bound.")
        # check nominal value (must be between bounds)
        if any(self._nominal_value < self._lower_bound) or any(
            self._upper_bound < self._nominal_value
        ):
            logger.error("Nominal value must be contained within bounds.")
            raise ValueError("Nominal value must be contained within bounds.")

    @property
    def nominal_value(self) -> np.ndarray:
        """
        Returns the nominal value, which is either the user-defined nominal value (passed as an input argument) or the mean of the uniform distribution
        :return: np.ndarray of dimension (self.dim,)
        """
        return self._nominal_value

    def sample_uncertainty(self) -> np.ndarray:
        """
        Generates a random sample from the uniform distribution.
        :return: np.ndarray of dimension (self.dim,)
        """
        return np.random.uniform(self._lower_bound, self._upper_bound)
