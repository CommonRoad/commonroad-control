import unittest

import numpy as np
from typing import Tuple
import math

from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import FullStateFeedback
from commonroad_control.simulation.sensor_models.sensor_model_interface import SensorModelInterface
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.simulation.simulation.simulation import Simulation
from commonroad_control.simulation.measurement_noise_model.measurement_noise_model import MeasurementNoiseModel
from commonroad_control.simulation.uncertainty_model.uniform_distribution import UniformDistribution


class TestSimulation(unittest.TestCase):

    @staticmethod
    def make_vehicle(
    ) -> Tuple[DynamicBicycle, DBSIDTFactory]:
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = DBSIDTFactory()
        vehicle_model_sim = DynamicBicycle(params=vehicle_params, delta_t=dt)

        return vehicle_model_sim, sit_factory_sim

    @staticmethod
    def problem_accelerate_steering_ramp(
    ) -> Tuple[DBState, DBInput, float]:
        # problem
        # ,,, initial state
        x0 = DBState(
            position_x=0.0,
            position_y=0.0,
            velocity_long=20.0,
            velocity_lat=0.0,
            heading=0.0,
            yaw_rate=0.0,
            steering_angle=0.0
        )
        # ... control input
        u_sim = DBInput(
            acceleration=0.5,
            steering_angle_velocity=0.1
        )
        # ... time horizon for simulation
        t_final_sim = 1.0

        return x0, u_sim, t_final_sim

    def standard_checks(self,
                       perturbed_trajectory: TrajectoryInterface,
                       nominal_trajectory: TrajectoryInterface,
                       x0: StateInterface,
                       num_steps_expected: int,
                       t_final_sim: float
                       ) -> None:
        """
        Standard test cases for the nominal and perturbed trajectory: check the number of time steps (identical to the
        given expected number), the horizon length (identical to t_final_sim) and the initial states (must be identical
        to x0).
        :param perturbed_trajectory: trajectory of the perturbed system
        :param nominal_trajectory: trajectory of the nominal system
        :param x0: initial state
        :param num_steps_expected: expected number of simulation time steps
        :param t_final_sim: simulation time horizon
        :return:
        """

        # ... final time horizon
        self.assertAlmostEqual(perturbed_trajectory.t_final, t_final_sim, places=10)
        self.assertAlmostEqual(nominal_trajectory.t_final, t_final_sim, places=10)
        # ... number of time steps
        self.assertEqual(len(perturbed_trajectory.steps), num_steps_expected+1)
        self.assertEqual(len(nominal_trajectory.steps), num_steps_expected+1)

        # ... initial states are identical
        self.assertLessEqual(np.linalg.norm(
            perturbed_trajectory.initial_point.convert_to_array() - x0.convert_to_array()), 1e-10)
        self.assertLessEqual(np.linalg.norm(
            nominal_trajectory.initial_point.convert_to_array() - x0.convert_to_array()), 1e-10)

    def test_sim_nominal(self):
        """
        Nominal simulation, i.e., no noise and no disturbances - thus, the final states must coincide for the nominal,
        perturbed, and measured trajectory.
        :return:
        """
        # get vehicle model
        vehicle_sim, sit_factory_sim = self.make_vehicle()

        # setup simulation
        sim_nom: Simulation = Simulation(
            vehicle_model=vehicle_sim,
            state_input_factory=sit_factory_sim,
            delta_t_sim=vehicle_sim._delta_t
        )

        # function under test
        # ... problem parameters
        x0, u_sim, t_final_sim = self.problem_accelerate_steering_ramp()
        # ... run simulation
        y_sim, x_sim_w, x_sim_nom = sim_nom.simulate(
            x0=x0,
            u=u_sim,
            t_final=t_final_sim
        )

        # check output args
        # ... standard tests
        num_steps_expected = math.ceil(t_final_sim/sim_nom._delta_t_sim)
        self.standard_checks(
            perturbed_trajectory=x_sim_w,
            nominal_trajectory=x_sim_nom,
            x0=x0,
            num_steps_expected=num_steps_expected,
            t_final_sim=t_final_sim
        )

        # ... final states are identical for the perturbed and nominal trajectory
        self.assertLessEqual(np.linalg.norm(
            x_sim_w.final_point.convert_to_array() - x_sim_nom.final_point.convert_to_array()), 1e-10)

        # ... since the default setting is full state feedback and no measurement noise, the measured output must be
        # identical to the final state of the perturbed trajectory
        self.assertLessEqual(np.linalg.norm(
            y_sim.convert_to_array() - x_sim_nom.final_point.convert_to_array()), 1e-10)

    def test_sim_perturbed(self):
        """
        Simulation of the perturbed system, noise: final state of the perturbed and measured trajectory must coincide.
        For testing, we choose the set of disturbances so that the origin is not contained - hence, the final state of
        the nominal trajectory must not coincide with the final state of the perturbed trajectory.
        :return:
        """
        # get vehicle model
        vehicle_sim, sit_factory_sim = self.make_vehicle()

        # setup simulation
        # ... disturbances
        nom_value = np.asarray([0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
        lb = 0.5*nom_value
        ub = 1.5*nom_value
        disturbance_model: UncertaintyModelInterface =UniformDistribution(
            dim=vehicle_sim.disturbance_dimension,
            lower_bound=lb,
            upper_bound=ub
        )

        # ... simulation
        sim_perturbed: Simulation = Simulation(
            vehicle_model=vehicle_sim,
            state_input_factory=sit_factory_sim,
            disturbance_model=disturbance_model,
            random_disturbance=True,
            delta_t_sim=vehicle_sim._delta_t
        )

        # function under test
        # ... problem parameters
        x0, u_sim, t_final_sim = self.problem_accelerate_steering_ramp()
        # ... run simulation
        y_sim, x_sim_w, x_sim_nom = sim_perturbed.simulate(
            x0=x0,
            u=u_sim,
            t_final=t_final_sim
        )

        # check output args
        # ... standard tests
        num_steps_expected = math.ceil(t_final_sim/sim_perturbed._delta_t_sim)
        self.standard_checks(
            perturbed_trajectory=x_sim_w,
            nominal_trajectory=x_sim_nom,
            x0=x0,
            num_steps_expected=num_steps_expected,
            t_final_sim=t_final_sim
        )

        # ... final state of perturbed trajectory must not coincide with nominal state (0 not contained in set of disturbances)
        self.assertGreater(np.linalg.norm(
            x_sim_w.final_point.convert_to_array() - x_sim_nom.final_point.convert_to_array()), 1e-4)

        # ... since the default setting is full state feedback and no measurement noise, the measured output must be
        # identical to the final state of the perturbed trajectory
        self.assertLessEqual(np.linalg.norm(
            y_sim.convert_to_array() - x_sim_w.final_point.convert_to_array()), 1e-10)

    def test_sim_perturbed_and_noisy(self):
        """
        Simulation of the perturbed system including measurement noise. For testing, we choose the set of disturbances
        and noises so that the origin is not contained. Hence, the final state of the nominal trajectory must not
        coincide with the final state of the perturbed trajectory, which must not coincide with the final state of the
        measured trajectory.
        :return:
        """
        # get vehicle model
        vehicle_sim, sit_factory_sim = self.make_vehicle()

        # setup simulation
        # ... disturbances
        nom_value = np.asarray([0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0])
        lb = 0.5*nom_value
        ub = 1.5*nom_value
        disturbance_model: UncertaintyModelInterface = UniformDistribution(
            dim=vehicle_sim.disturbance_dimension,
            lower_bound=lb,
            upper_bound=ub
        )
        # ... noise
        nom_value = np.asarray([0.075, 0.075, 0.0, 0.0, 0.0, 0.0, 0.0])
        lb = 0.5*nom_value
        ub = 1.5*nom_value
        noise_model: UniformDistribution = UniformDistribution(
            dim=vehicle_sim.state_dimension,
            lower_bound=lb,
            upper_bound=ub
        )
        sensor_model: FullStateFeedback = FullStateFeedback(
            state_output_factory=sit_factory_sim,
            noise_model=noise_model,
            state_dimension=vehicle_sim.state_dimension,
            input_dimension=vehicle_sim.input_dimension
        )
        # ... simulation
        sim_noisy: Simulation = Simulation(
            vehicle_model=vehicle_sim,
            state_input_factory=sit_factory_sim,
            disturbance_model=disturbance_model,
            random_disturbance=True,
            sensor_model=sensor_model,
            random_noise=True,
            delta_t_sim=vehicle_sim._delta_t
        )

        # function under test
        # ... problem parameters
        x0, u_sim, t_final_sim = self.problem_accelerate_steering_ramp()
        # ... run simulation
        y_sim, x_sim_w, x_sim_nom = sim_noisy.simulate(
            x0=x0,
            u=u_sim,
            t_final=t_final_sim
        )

        # check output args
        # ... standard tests
        num_steps_expected = math.ceil(t_final_sim/sim_noisy._delta_t_sim)
        self.standard_checks(
            perturbed_trajectory=x_sim_w,
            nominal_trajectory=x_sim_nom,
            x0=x0,
            num_steps_expected=num_steps_expected,
            t_final_sim=t_final_sim
        )

        # ... final state of perturbed and nominal trajectory must not coincide (0 not contained in set)
        self.assertGreater(np.linalg.norm(
            x_sim_w.final_point.convert_to_array() - x_sim_nom.final_point.convert_to_array()), 1e-4)

        # ... final state of perturbed and measured state (we assume full state feedback) must not coincide (0 not contained in set of noises)
        self.assertGreater(np.linalg.norm(
            x_sim_w.final_point.convert_to_array() - y_sim.convert_to_array()), 1e-6)
