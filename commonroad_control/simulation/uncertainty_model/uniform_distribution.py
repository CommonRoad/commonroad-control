from typing import Union
import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface


class UniformDistribution(UncertaintyModelInterface):
    def __init__(
            self,
            dim: int,
            lower_bound: Union[np.ndarray, list[float]],
            upper_bound: Union[np.ndarray, list[float]],
            nominal_value: Union[np.ndarray, list[float]] = None
    ) -> None:

        super().__init__(dim=dim)

        self._lower_bound: np.ndarray = np.array(lower_bound)
        self._upper_bound: np.ndarray = np.array(upper_bound)

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
            raise ValueError(
                f"Dimension mismatch: "
                f"expected dimension:{self._dim}, "
                f"lower bound:{len(self._lower_bound)}, upper bound:{len(self._upper_bound)}"
            )
        # check bounds (lb <= ub)
        if any(self._upper_bound < self._lower_bound):
            raise ValueError(
                f"Upper bound must be greater than lower bound."
            )
        # check nominal value (must be between bounds)
        if any(self._nominal_value < self._lower_bound) or any(self._upper_bound < self._nominal_value):
            raise ValueError(
                f"Nominal value must be contained within bounds."
            )

    @property
    def nominal_value(self) -> np.ndarray:
        return self._nominal_value

    def sample_uncertainty(self):
        return np.random.uniform(self._lower_bound, self._upper_bound)
