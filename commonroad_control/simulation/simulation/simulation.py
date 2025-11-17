from typing import Union, Any, Optional, Tuple

import numpy as np
import math
from scipy.integrate import solve_ivp, OdeSolver

from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface, TrajectoryMode
from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface

from commonroad_control.simulation.measurement_noise_model.measurement_noise_model import MeasurementNoiseModel
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.simulation.uncertainty_model.no_uncertainty import NoUncertainty

class Simulation:
    """
    Class for the simulation of a dynamic system.
    """
    def __init__(
            self,
            vehicle_model: VehicleModelInterface,
            state_input_factory: StateInputTrajectoryFactoryInterface,
            disturbance_model: Optional[UncertaintyModelInterface] = None,
            random_disturbance: Optional[bool] = False,
            noise_model: Optional[MeasurementNoiseModel] = None,
            random_noise: Optional[bool] = False,
            delta_t_sim: Optional[float] = 0.1,
    ) -> None:
        """
        Simulates a dynamical system given an initial state and a (constant) control input for a given time horizon with
        optional disturbances. By using the vehicle_model_interface and state_input_factory, the simulation can
        automatically deduce the state and input types as well as the system of differential equations modelling the
        system dynamics.
        :param vehicle_model: vehicle model interface for simulation, e.g. kinematic bicycle model
        :param state_input_factory: object that can generate/convert states and inputs for a given vehicle model
        :param disturbance_model: optional uncertainty model for generating (random) disturbance values
        :param random_disturbance: true if random values shall be sampled from the disturbance model
        :param noise_model: optional uncertainty model for generating and applying (random) noise values
        :param random_noise: true if random values shall be sampled form the noise uncertainty model
        :param delta_t_sim: (max.) simulation time step, also used for sampling the disturbance
        """

        self._vehicle_model: VehicleModelInterface = vehicle_model
        self._state_input_factory: StateInputTrajectoryFactoryInterface = state_input_factory
        self._delta_t_sim: Optional[float] = delta_t_sim

        # set disturbance model
        self._random_disturbance: bool = random_disturbance if disturbance_model is not None else False
        # ... if none is provided, set default model (no uncertainty)
        if disturbance_model is None:
            disturbance_model: UncertaintyModelInterface = NoUncertainty(
                    dim=self._vehicle_model.disturbance_dimension
            )
        self._disturbance_model: UncertaintyModelInterface = disturbance_model

        # set noise model
        self._random_noise: bool = random_noise if noise_model is not None else False
        self._noise_model: Optional[MeasurementNoiseModel] = noise_model

    @property
    def vehicle_model(self) -> VehicleModelInterface:
        """
        :return: vehicle model interface
        """
        return self._vehicle_model

    @property
    def state_input_factory(self) -> StateInputTrajectoryFactoryInterface:
        """
        :return: state input factory
        """
        return self._state_input_factory

    @property
    def disturbance_model(self) -> Optional[UncertaintyModelInterface]:
        """
        :return: uncertainty model for generating (random) disturbance values
        """
        return self._disturbance_model

    @property
    def random_disturbance(self) -> bool:
        """
        :return: true, if random values are sampled from the disturbance uncertainty model, otherwise, its nominal value
        is applied
        """
        return self._random_disturbance

    @property
    def noise_model(self) -> Optional[MeasurementNoiseModel]:
        """
        :return: uncertainty model for generating and applying (random) noise values
        """
        return self._noise_model

    @property
    def random_noise(self) -> bool:
        """
        :return: True if random values are sampled from the noise uncertainty model and applied to the simualted state
        """
        return self._random_noise

    def simulate(
            self,
            x0: StateInterface,
            u: InputInterface,
            t_final: float,
            ivp_method: Union[str, OdeSolver, None] = "RK45",
    ) -> Tuple[TrajectoryInterface, TrajectoryInterface, TrajectoryInterface]:
        """
        Simulates the dynamical system starting from the initial state x0 until time t_final. The control input is kept
        constant for t in [0, t_final]. The default method for solving the initial value problem is RK45.
        The value of the (optional) disturbance is piece-wise constant and re-sampled (the latest) every
        self._delta_t_sim seconds.
        :param x0: initial state
        :param u: control input
        :param t_final: final time for simulation (assuming initial time is 0)
        :param ivp_method: method for solving the initial value problem.
        :return: Tuple[perturbed and noisy trajectory, perturbed trajectory, nominal trajectory]
        """

        x0_np: np.ndarray = x0.convert_to_array()
        u_np: np.ndarray = u.convert_to_array()

        # initialize simulation time and state
        t_sim: float = 0
        x_sim_nom: dict = {0: x0_np}
        x_sim_w: dict = {0: x0_np}
        x_sim_noise: dict = {0: x0_np}

        # compute time step size (< self._delta_t_sim) - trajectory interface only allows evenly spaced time horizons
        num_step_sim = math.ceil(t_final/self._delta_t_sim)
        delta_t_sim = t_final / num_step_sim

        for kk in range(num_step_sim):
            # duration of simulation step
            delta_t = max([min([t_final - t_sim, delta_t_sim]),0])

            # simulate nominal system
            w_np_nom: np.ndarray = self._disturbance_model.nominal_value
            res_sim_nom = solve_ivp(
                lambda t, y: self._vehicle_model.dynamics_ct(y, u_np, w_np_nom),
                [0, delta_t_sim],
                y0=x_sim_nom[kk],
                method=ivp_method
            )

            # simulate perturbed system
            # ... sample disturbance
            w_np: np.ndarray = (
                self._disturbance_model.sample_uncertainty()) if self._random_disturbance \
                else self._disturbance_model.nominal_value
            # ... simulate
            res_sim_w = solve_ivp(
                lambda t, y: self._vehicle_model.dynamics_ct(y, u_np, w_np),
                [0, delta_t_sim],
                y0=x_sim_w[kk],
                method=ivp_method
            )

            # extract result and update time
            x_sim_nom[kk+1] = res_sim_nom.y[:, -1]
            x_sim_w[kk+1] = res_sim_w.y[:, -1]

        # apply noise
        x_sim_noise[1] = self._noise_model.apply_noise(x_sim_w[num_step_sim],apply_rand_noise=True) \
            if self._random_noise else x_sim_w[num_step_sim]

        # output arguments
        x_sim_nom: TrajectoryInterface = self._state_input_factory.trajectory_from_numpy_array(
            traj_np=np.column_stack(list(x_sim_nom.values())),
            mode=TrajectoryMode.State,
            time_steps=list(x_sim_nom.keys()),
            t_0=0.0,
            delta_t=delta_t_sim

        )
        x_sim_w: TrajectoryInterface = self._state_input_factory.trajectory_from_numpy_array(
            traj_np=np.column_stack(list(x_sim_w.values())),
            mode=TrajectoryMode.State,
            time_steps=list(x_sim_w.keys()),
            t_0=0.0,
            delta_t=delta_t_sim

        )
        x_sim_noise: TrajectoryInterface = self._state_input_factory.trajectory_from_numpy_array(
            traj_np=np.column_stack(list(x_sim_noise.values())),
            mode=TrajectoryMode.State,
            time_steps=list(x_sim_noise.keys()),
            t_0=0.0,
            delta_t=t_final

        )

        return x_sim_noise, x_sim_w, x_sim_nom
