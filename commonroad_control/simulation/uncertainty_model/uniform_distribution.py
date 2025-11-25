from typing import Union
import numpy as np
import logging

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.simulation.uncertainty_model.uncertainty_interface import UncertaintyInterface

logger = logging.getLogger(__name__)

class UniformDistribution(UncertaintyModelInterface):
    def __init__(
            self,
            dim: int,
            lower_bound: Union[np.ndarray, list[float], UncertaintyInterface],
            upper_bound: Union[np.ndarray, list[float], UncertaintyInterface],
            *args,
            nominal_value: Union[np.ndarray, list[float], UncertaintyInterface] = None,
            **kwargs
    ) -> None:

        super().__init__(dim=dim)

        if isinstance(lower_bound, UncertaintyInterface):
            lower_bound_np = lower_bound.convert_to_array()
        else:
            lower_bound_np : np.ndarray = np.array(lower_bound)
        self._lower_bound: np.ndarray = lower_bound_np

        if isinstance(upper_bound, UncertaintyInterface):
            upper_bound_np = upper_bound.convert_to_array()
        else:
            upper_bound_np : np.ndarray = np.array(upper_bound)
        self._upper_bound: np.ndarray = upper_bound_np

        # set nominal value
        if nominal_value is not None:
            self._nominal_value = nominal_value
        else:
            self._nominal_value = 0.5*(upper_bound + lower_bound)

        self._sanity_check()

    def _sanity_check(self) -> None:
        """
        Check args.
        :return:
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
            logger.error(
                f"Upper bound must be greater than lower bound."
            )
            raise ValueError(
                f"Upper bound must be greater than lower bound."
            )
        # check nominal value (must be between bounds)
        if any(self._nominal_value < self._lower_bound) or any(self._upper_bound < self._nominal_value):
            logger.error(
                f"Nominal value must be contained within bounds."
            )
            raise ValueError(
                f"Nominal value must be contained within bounds."
            )

    @property
    def nominal_value(self) -> np.ndarray:
        return self._nominal_value

    def sample_uncertainty(self):
        return np.random.uniform(self._lower_bound, self._upper_bound)
