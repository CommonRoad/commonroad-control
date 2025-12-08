import logging
from typing import Union

import casadi as cas
import numpy as np

from commonroad_control.simulation.sensor_models.sensor_model_interface import (
    SensorModelInterface,
)
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface

logger = logging.getLogger(__name__)


class FullStateFeedback(SensorModelInterface):
    """
    Full state feedback sensor model, i.e., the output function is y = x + n, where n denotes (random) measurement noise.
    """

    def __init__(
        self,
        noise_model: UncertaintyModelInterface,
        state_output_factory: StateInputDisturbanceTrajectoryFactoryInterface,
        state_dimension: int,
        input_dimension: int,
    ):
        """
        Initialize sensor model.
        :param noise_model: uncertainty model representing sensor noise - UncertaintyModelInterface
        :param state_output_factory: factory for creating States or Outputs as output arguments - StateInputDisturbanceTrajectoryFactoryInterface
        :param state_dimension: state dimension - int
        :param input_dimension: input dimension - int
        """

        # init base class
        super().__init__(
            noise_model=noise_model,
            state_output_factory=state_output_factory,
            dim=state_dimension,
            state_dimension=state_dimension,
            input_dimension=input_dimension,
        )

        # sanity check
        self._sanity_check()

    def _output_function_cas(
        self, x: Union[np.array, cas.SX.sym], u: Union[np.array, cas.SX.sym]
    ) -> Union[cas.SX.sym, np.array]:
        """
        Implements the nominal output function y=x.
        :param x: state
        :param u: control input
        :return: value of the output function evaluated at x, u
        """
        return x

    def _sanity_check(self):
        """
        Check args.
        """

        # dimension of noise model must match dimension of the output
        if self._noise_model.dim != self._dim != self.state_dimension:
            logger.error(
                "Dimensions of noise, output, and state do not match but must be identical."
            )
            raise ValueError(
                "Dimensions of noise, output, and state do not match but must be identical."
            )
        # state_output_factory is of correct type
        if not isinstance(
            self._state_output_factory, StateInputDisturbanceTrajectoryFactoryInterface
        ):
            logger.error(
                f"x must be of type {StateInputDisturbanceTrajectoryFactoryInterface.__name__}, "
                f"not {type(self._state_output_factory).__name__}"
            )
            raise TypeError(
                f"x must be of type {StateInputDisturbanceTrajectoryFactoryInterface.__name__}, "
                f"not {type(self._state_output_factory).__name__}"
            )
        # state dimension of state_output_factory must match state dimension
        if self._dim != self._state_output_factory.state_dimension:
            logger.error(
                "Dimension of output does not match the dimension of the state/output factory."
            )
            raise ValueError(
                "Dimension of output does not match the dimension of the state/output factory."
            )

    def measure(
        self, x: StateInterface, u: InputInterface, rand_noise: bool = True
    ) -> StateInterface:
        """
        Evaluates the output function and applies (random) noise to the output.
        :param x: state
        :param u: input
        :param rand_noise: true, if random noise shall be applied, otherwise the nominal value of the uncertainty model is applied.
        :return: (noisy) measurement
        """

        # check input arguments
        if x.dim != self._state_dimension:
            logger.error(f"Dimension of state {x.dim} does not match.")
            raise ValueError(f"Dimension of state {x.dim} does not match.")
        if u.dim != self._input_dimension:
            logger.error(f"Dimension of input {u.dim} does not match.")
            raise ValueError(f"Dimension of input {u.dim} does not match.")

        # evaluate nominal output
        y_nom_np = self._nominal_output(x, u)

        # sample and apply noise
        if rand_noise:
            noise_np = self._noise_model.sample_uncertainty()
        else:
            noise_np = self._noise_model.nominal_value
        y_np = y_nom_np + noise_np

        # instantiate state interface object as output
        y = self._state_output_factory.state_from_numpy_array(y_np)

        return y
