from abc import ABC, abstractmethod
from typing import Any, Union

import casadi as cas
import numpy as np

from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface


class SensorModelInterface(ABC):
    """
    Interface for sensor models which take the current state and control input as an input argument and simulate a (noisy) measurement.
    """

    def __init__(
        self,
        noise_model: UncertaintyModelInterface,
        state_output_factory: Union[
            StateInputDisturbanceTrajectoryFactoryInterface, Any
        ],
        dim: int,
        state_dimension: int,
        input_dimension: int,
    ):
        """
        Initialize baseclass.
        :param noise_model: uncertainty model representing sensor noise - UncertaintyModelInterface
        :param state_output_factory: factory for creating States or Outputs as output arguments - StateInputDisturbanceTrajectoryFactoryInterface
        :param dim: dimension of the output - int
        :param state_dimension: state dimension - int
        :param input_dimension: input dimension - int
        """
        self._noise_model: UncertaintyModelInterface = noise_model
        self._state_output_factory: Union[
            StateInputDisturbanceTrajectoryFactoryInterface, Any
        ] = state_output_factory
        self._dim: int = dim
        self._state_dimension: int = state_dimension
        self._input_dimension: int = input_dimension

        # setup casadi function wrapping the nominal output function
        xk = cas.SX.sym("xk", state_output_factory.state_dimension, 1)
        uk = cas.SX.sym("uk", state_output_factory.input_dimension, 1)
        self._output_function = cas.Function(
            "output_function", [xk, uk], [self._output_function_cas(xk, uk)]
        )

    @property
    def noise_model(self) -> UncertaintyModelInterface:
        """
        :return: Uncertainty model representing sensor noise
        """
        return self._noise_model

    @property
    def output_dimension(self) -> int:
        """
        :return: output dimension
        """
        return self._dim

    @property
    def state_dimension(self) -> int:
        """
        :return: state dimension
        """
        return self._state_dimension

    @property
    def input_dimension(self) -> int:
        """
        :return: input dimension
        """
        return self._input_dimension

    @abstractmethod
    def _output_function_cas(
        self, x: Union[np.array, cas.SX.sym], u: Union[np.array, cas.SX.sym]
    ) -> Union[cas.SX.sym, np.array]:
        """
        Implements the nominal output function y=h(x,u), where e.g., h(x,u) = C*x + D*u for linear systems.
        :param x: state
        :param u: control input
        :return: value of the output function evaluated at x, u
        """
        pass

    def nominal_output(
        self, x: Union[StateInterface, np.array], u: Union[StateInterface, np.array]
    ) -> np.ndarray:
        """
        Evaluates the nominal output value given a state and corresponding control input.
        :param x: state
        :param u: control input
        :return: nominal output at (x,u) represented as a numpy array
        """

        # convert state and input to arrays
        if isinstance(x, StateInterface):
            x_np = x.convert_to_array()
        else:
            x_np = x

        if isinstance(u, InputInterface):
            u_np = u.convert_to_array()
        else:
            u_np = u

        # evaluate output function
        x_next = self._output_function(x_np, u_np).full()

        return x_next.squeeze()

    @abstractmethod
    def measure(
        self, x: StateInterface, u: InputInterface, rand_noise: bool = True
    ) -> Union[StateInterface, Any]:
        """
        Evaluates the output function and applies (random) noise to the output.
        :param x: state
        :param u: input
        :param rand_noise: true, if random noise shall be applied, otherwise the nominal value of the uncertainty model is applied.
        :return: (noisy) measurement
        """
        pass
