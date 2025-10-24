from typing import Union, Any, Optional, Tuple

import numpy as np
from scipy.integrate import solve_ivp, OdeSolver

from commonroad_control.noise_disturbance.NoiseDisturbanceGeneratorInterface import NoiseDisturbanceGeneratorInterface
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class Simulation:
    """
    Class for the simulation of a dynamic system.
    """
    def __init__(
            self,
            vehicle_model: VehicleModelInterface,
            state_input_factory: StateInputTrajectoryFactoryInterface,
            disturbance_generator: Optional[NoiseDisturbanceGeneratorInterface] = None,
            noise_generator: Optional[NoiseDisturbanceGeneratorInterface] = None
    ) -> None:
        """
        Simulates a dynamic system given state and input for a given time horizon with optional disturbances.
        By using the vehicle_model and state_input_factory, the simulation can automatically deduce the state and input
        types as well as the dynamic equations
        :param vehicle_model: vehicle model for the simulation, e.g. Kinematic Single Track
        :param state_input_factory: object that can generate/convert states and inputs for a given vehicle model
        :param disturbance_generator: optional disturbance generator that applies disturbance to the state after forward simulation
        :param noise_generator: optional noise generator that applies noise to state after disturbance
        """
        self._vehicle_model: VehicleModelInterface = vehicle_model
        self._state_input_factory: StateInputTrajectoryFactoryInterface = state_input_factory
        self._disturbance_generator: Optional[NoiseDisturbanceGeneratorInterface] = disturbance_generator
        self._uses_disturbances: bool = True if disturbance_generator is not None else False
        self._noise_generator: Optional[NoiseDisturbanceGeneratorInterface] = noise_generator
        self._uses_noise: bool = True if noise_generator is not None else False

    @property
    def vehicle_model(self) -> VehicleModelInterface:
        """
        :return: vehicle model
        """
        return self._vehicle_model

    @property
    def state_input_factory(self) -> StateInputTrajectoryFactoryInterface:
        """
        :return: state input factory
        """
        return self._state_input_factory

    @property
    def disturbance_generator(self) -> Optional[NoiseDisturbanceGeneratorInterface]:
        """
        :return: noise generator
        """
        return self._disturbance_generator

    @property
    def uses_disturbances(self) -> bool:
        """
        :return: true, if disturbances are applied after forward simulation
        """
        return self._uses_disturbances

    @property
    def noise_generator(self) -> Optional[NoiseDisturbanceGeneratorInterface]:
        """
        :return: noise generator
        """
        return self._noise_generator

    @property
    def uses_noise(self) -> bool:
        """
        :return: True if noise was applied after forward simulation
        """
        return self._uses_noise

    def simulate(
        self,
        x0: StateInterface,
        u: InputInterface,
        time_horizon: float,
        ivp_method: Union[str, OdeSolver, None] = "RK45"
    ) -> Tuple[Union[StateInterface, Any], Union[StateInterface, Any], Union[StateInterface, Any]]:
        """
        Simulates the current state forward given an input and the time horizon using Runge-Kutta 4.
        :param x0: current state
        :param u: input
        :param time_horizon: number of seconds to simulate forward
        :return: Tuple[state_with_disturbance_noise, state_with_disturbance_no_noise, state_no_disturbance_no_noise]
        """
        x0_num = x0.convert_to_array()
        u_num = u.convert_to_array()

        # TODO fix to public var
        res_sim = solve_ivp(
            lambda t, y: self._vehicle_model._dynamics_ct(y, u_num),
            [0, time_horizon],
            y0=x0_num,
            method=ivp_method
        )
        x_end = res_sim.y[:, -1]


        # Add disturbance
        x_disturbed: np.ndarray = self._disturbance_generator.apply(x_end) if self._uses_disturbances else x_end

        # Add noise
        x_noise: np.ndarray = self._noise_generator.apply(x_disturbed) if self._uses_noise else x_disturbed

        x_forward_disturbed_noisy = self._state_input_factory.state_from_numpy_array(x_np=x_noise)
        x_forward_disturbed_no_noise = self._state_input_factory.state_from_numpy_array(x_np=x_disturbed)
        x_forward_no_disturbance_no_noise = self._state_input_factory.state_from_numpy_array(x_np=x_end)

        return x_forward_disturbed_noisy, x_forward_disturbed_no_noise, x_forward_no_disturbance_no_noise
