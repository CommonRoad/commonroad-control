import unittest

import numpy as np
from typing import Tuple
import math

from commonroad_control.simulation.sensor_models.full_state_feedback.full_state_feedback import FullStateFeedback
from commonroad_control.simulation.uncertainty_model.uncertainty_model_interface import UncertaintyModelInterface
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput
from commonroad_control.vehicle_dynamics.double_integrator.di_noise import DINoise
from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import DISIDTFactory
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState
from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_noise import DBNoise
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_noise import KBNoise
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import KBSIDTFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.simulation.simulation.simulation import Simulation
from commonroad_control.simulation.uncertainty_model.uniform_distribution import UniformDistribution


class TestVehicleModelsSimulation(unittest.TestCase):
    """
    Simulation tests for all vehicle models subject to disturbances with noisy full state feedback sensor.
    For testing, we choose the set of disturbances and noises so that the origin is not contained. Hence, the final state of the nominal trajectory must not coincide with the final state of the perturbed trajectory, which must not coincide with the final state of the measured trajectory.
    """

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

    def test_sim_double_integrator(self):
        """
        Simulation of the double integrator model.
        """
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = DISIDTFactory()
        vehicle_sim = DoubleIntegrator(params=vehicle_params, delta_t=dt)

        # setup simulation
        # ... disturbances
        nom_value = DISIDTFactory.disturbance_from_args(
            position_long=0.0,
            position_lat=0.0,
            velocity_long=0.05,
            velocity_lat=0.05
        )
        lb = 0.5 * nom_value.convert_to_array()
        ub = 1.5 * nom_value.convert_to_array()
        disturbance_model: UncertaintyModelInterface = UniformDistribution(
            dim=vehicle_sim.disturbance_dimension,
            lower_bound=DISIDTFactory.disturbance_from_numpy_array(lb),
            upper_bound=DISIDTFactory.disturbance_from_numpy_array(ub)
        )
        # ... noise
        nom_value = DINoise(
            position_long=0.075,
            position_lat=0.075
        )
        lb = 0.5 * nom_value.convert_to_array()
        ub = 1.5 * nom_value.convert_to_array()
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

        # problem statement - constant acceleration in long. and lat. direction
        # ... initial state
        x0 = DIState(
            position_long=0.0,
            position_lat=0.0,
            velocity_long=20.0,
            velocity_lat=0.0,
        )
        # ... control input
        u_sim = DIInput(
            acceleration_long=0.5,
            acceleration_lat=0.1
        )
        # ... time horizon for simulation
        t_final_sim = 1.0

        # function under test
        # ... run simulation
        y_sim, x_sim_w, x_sim_nom = sim_noisy.simulate(
            x0=x0,
            u=u_sim,
            t_final=t_final_sim
        )

        # check output args
        # ... standard tests
        num_steps_expected = math.ceil(t_final_sim / sim_noisy._delta_t_sim)
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

    def test_sim_kinematic_bicycle(self):
        """
        Simulation of the kinematic bicycle model.
        """
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = KBSIDTFactory()
        vehicle_sim = KinematicBicycle(params=vehicle_params, delta_t=dt)

        # setup simulation
        # ... disturbances
        nom_value = KBSIDTFactory.disturbance_from_args(
            velocity=0.05,
            heading=0.05
        )
        lb = 0.5 * nom_value.convert_to_array()
        ub = 1.5 * nom_value.convert_to_array()
        disturbance_model: UncertaintyModelInterface = UniformDistribution(
            dim=vehicle_sim.disturbance_dimension,
            lower_bound=KBSIDTFactory.disturbance_from_numpy_array(lb),
            upper_bound=KBSIDTFactory.disturbance_from_numpy_array(ub)
        )
        # ... noise
        nom_value = KBNoise(
            position_x=0.075,
            position_y=0.075
        )
        lb = 0.5 * nom_value.convert_to_array()
        ub = 1.5 * nom_value.convert_to_array()
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

        # problem statement - constant acceleration with steering angle ramp
        # ... initial state
        x0 = KBState(
            position_x=0.0,
            position_y=0.0,
            velocity=20.0,
            heading=0.0,
            steering_angle=0.0
        )
        # ... control input
        u_sim = KBInput(
            acceleration=0.5,
            steering_angle_velocity=0.1
        )
        # ... time horizon for simulation
        t_final_sim = 1.0

        # function under test
        # ... run simulation
        y_sim, x_sim_w, x_sim_nom = sim_noisy.simulate(
            x0=x0,
            u=u_sim,
            t_final=t_final_sim
        )

        # check output args
        # ... standard tests
        num_steps_expected = math.ceil(t_final_sim / sim_noisy._delta_t_sim)
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

    def test_sim_dynamic_bicycle(self):
        """
        Simulation of the dynamic bicycle model.
        """
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = DBSIDTFactory()
        vehicle_sim = DynamicBicycle(params=vehicle_params, delta_t=dt)

        # setup simulation
        # ... disturbances
        nom_value = DBSIDTFactory.disturbance_from_args(
            velocity_long=0.05,
            velocity_lat=0.05
        )
        lb = 0.5*nom_value.convert_to_array()
        ub = 1.5*nom_value.convert_to_array()
        disturbance_model: UncertaintyModelInterface = UniformDistribution(
            dim=vehicle_sim.disturbance_dimension,
            lower_bound=lb,
            upper_bound=ub
        )
        # ... noise
        nom_value = DBNoise(
            position_x=0.075,
            position_y=0.075
        )
        lb = 0.5*nom_value.convert_to_array()
        ub = 1.5*nom_value.convert_to_array()
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

        # problem statement - constant acceleration with steering angle ramp
        # ... initial state
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

        # function under test
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
