import math
from typing import Optional, Tuple, Union

import numpy as np
from scipy.integrate import OdeSolver, solve_ivp

from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import (
    FullStateFeedback,
)
from commonroad_control.simulation.sensor_models.output_interface import OutputInterface
from commonroad_control.simulation.sensor_models.sensor_model_interface import (
    SensorModelInterface,
)
from commonroad_control.simulation.uncertainty_model.no_uncertainty import NoUncertainty
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import (
    UncertaintyModelInterface,
)
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.sidt_factory_interface import (
    StateInputDisturbanceTrajectoryFactoryInterface,
)
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.trajectory_interface import (
    TrajectoryInterface,
    TrajectoryMode,
)
from commonroad_control.vehicle_dynamics.vehicle_model_interface import (
    VehicleModelInterface,
)


class Simulation:
    """
    Class for the simulation of a dynamic system.
    """

    def __init__(
        self,
        vehicle_model: VehicleModelInterface,
        sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface,
        disturbance_model: Optional[UncertaintyModelInterface] = None,
        random_disturbance: Optional[bool] = False,
        sensor_model: Optional[SensorModelInterface] = None,
        random_noise: Optional[bool] = False,
        delta_t_w: Optional[float] = 0.1,
    ) -> None:
        """
        Simulates a dynamical system given an initial state and a (constant) control input for a given time horizon with
        optional disturbances. By using the vehicle_model_interface and sidt_factory, the simulation can
        automatically deduce the state and input types as well as the system of differential equations modeling the
        system dynamics.
        Output functions and measurement noise can be considered via an optional sensor model.
        :param vehicle_model: vehicle model interface for simulation, e.g. kinematic bicycle model
        :param sidt_factory: object that can generate/convert states and inputs for a given vehicle model
        :param disturbance_model: optional uncertainty model for generating (random) disturbance values
        :param random_disturbance: true if random values shall be sampled from the disturbance model
        :param sensor_model: optional sensor model for computing (noisy) outputs
        :param random_noise: true if random noise shall be applied
        :param delta_t_w: time step size for sampling the disturbances
        """

        self._vehicle_model: VehicleModelInterface = vehicle_model
        self._sidt_factory: StateInputDisturbanceTrajectoryFactoryInterface = sidt_factory
        self._delta_t_w: Optional[float] = delta_t_w

        # set disturbance model
        self._random_disturbance: bool = random_disturbance if disturbance_model is not None else False
        # ... if none is provided, set default model (no uncertainty)
        if disturbance_model is None:
            disturbance_model: UncertaintyModelInterface = NoUncertainty(dim=self._vehicle_model.disturbance_dimension)
        self._disturbance_model: UncertaintyModelInterface = disturbance_model

        # set sensor model
        self._random_noise: bool = random_noise if sensor_model is not None else False
        # ... if none is provided, set default model (full state feedback with no uncertainty
        if sensor_model is None:
            sensor_model: SensorModelInterface = FullStateFeedback(
                noise_model=NoUncertainty(dim=self._vehicle_model.state_dimension),
                state_output_factory=self._sidt_factory,
                state_dimension=sidt_factory.state_dimension,
                input_dimension=sidt_factory.input_dimension,
            )
        self._sensor_model: Optional[SensorModelInterface] = sensor_model

    @property
    def vehicle_model(self) -> VehicleModelInterface:
        """
        :return: vehicle model interface
        """
        return self._vehicle_model

    @property
    def state_input_factory(self) -> StateInputDisturbanceTrajectoryFactoryInterface:
        """
        :return: state input factory
        """
        return self._sidt_factory

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
    def sensor_model(self) -> Optional[SensorModelInterface]:
        """
        :return: sensor model for computing (noisy) outputs
        """
        return self._sensor_model

    @property
    def random_noise(self) -> bool:
        """
        :return: True if random noise is applied to the measured state or output
        """
        return self._random_noise

    def simulate(
        self,
        x0: StateInterface,
        u: InputInterface,
        t_final: float,
        ivp_method: Union[str, OdeSolver, None] = "RK45",
    ) -> Tuple[Union[StateInterface, OutputInterface], TrajectoryInterface, TrajectoryInterface]:
        """
        Simulates the dynamical system starting from the initial state x0 until time t_final. The control input is kept
        constant for t in [0, t_final]. The default method for solving the initial value problem is RK45.
        The value of the (optional) disturbance is piece-wise constant and re-sampled (the latest) every
        self._delta_t_w seconds.
        :param x0: initial state
        :param u: control input
        :param t_final: final time for simulation (assuming initial time is 0)
        :param ivp_method: method for solving the initial value problem.
        :return: Tuple[noisy measurement (from perturbed trajectory), perturbed trajectory, nominal trajectory]
        """

        x0_np: np.ndarray = x0.convert_to_array()
        u_np: np.ndarray = u.convert_to_array()

        # initialize simulated trajectory
        x_sim_nom: dict = {0: x0_np}
        x_sim_w: dict = {0: x0_np}

        # compute time step size (< self._delta_t_w) - trajectory interface only allows evenly spaced time horizons
        num_step_sim = math.ceil(t_final / self._delta_t_w)
        delta_t_sim = t_final / num_step_sim

        for kk in range(num_step_sim):

            # simulate nominal system
            w_np_nom: np.ndarray = self._disturbance_model.nominal_value
            res_sim_nom = solve_ivp(
                lambda t, y: self._vehicle_model.dynamics_ct(y, u_np, w_np_nom),
                [0, delta_t_sim],
                y0=x_sim_nom[kk],
                method=ivp_method,
            )

            # simulate perturbed system
            # ... sample disturbance
            w_np: np.ndarray = (
                (self._disturbance_model.sample_uncertainty())
                if self._random_disturbance
                else self._disturbance_model.nominal_value
            )
            # ... simulate
            res_sim_w = solve_ivp(
                lambda t, y: self._vehicle_model.dynamics_ct(y, u_np, w_np),
                [0, delta_t_sim],
                y0=x_sim_w[kk],
                method=ivp_method,
            )

            # extract result and update time
            x_sim_nom[kk + 1] = res_sim_nom.y[:, -1]
            x_sim_w[kk + 1] = res_sim_w.y[:, -1]

        # compute output and apply noise
        # ... for causality, we pass the control input applied during simulation
        x_final = self._sidt_factory.state_from_numpy_array(x_sim_w[num_step_sim])
        y_sim_noise: StateInterface = self._sensor_model.measure(x_final, u, rand_noise=self._random_noise)

        # output arguments
        x_sim_nom: TrajectoryInterface = self._sidt_factory.trajectory_from_numpy_array(
            traj_np=np.column_stack(list(x_sim_nom.values())),
            mode=TrajectoryMode.State,
            time_steps=list(x_sim_nom.keys()),
            t_0=0.0,
            delta_t=delta_t_sim,
        )
        x_sim_w: TrajectoryInterface = self._sidt_factory.trajectory_from_numpy_array(
            traj_np=np.column_stack(list(x_sim_w.values())),
            mode=TrajectoryMode.State,
            time_steps=list(x_sim_w.keys()),
            t_0=0.0,
            delta_t=delta_t_sim,
        )

        return y_sim_noise, x_sim_w, x_sim_nom
