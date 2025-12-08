import numpy as np
import unittest
from typing import Tuple
import logging
from math import cos, sin, asin, tan, atan

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_dynamics.double_integrator.di_trajectory import DITrajectory
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput,DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_disturbance import (
    DIDisturbance,DIDisturbanceIndices
)
from commonroad_control.vehicle_dynamics.double_integrator.di_noise import DINoise
from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import DISIDTFactory


logger_global = configure_toolbox_logging(level=logging.DEBUG)


class TestDoubleIntegrator(unittest.TestCase):
    """
    Tests for double integrator bicycle model:
        - vehicle model:
            - instantiation
            - dynamics
            - normalized acceleration
        - sidt_factory 
        - (querying) trajectory
    """

    @staticmethod
    def make_vehicle(
    ) -> Tuple[DoubleIntegrator, DISIDTFactory]:
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = DISIDTFactory()
        vehicle_model_sim = DoubleIntegrator(params=vehicle_params, delta_t=dt)

        return vehicle_model_sim, sit_factory_sim

    def test_di_instanation(self) -> None:
        """
        Test correct instantiation of model.
        """
        vehicle_params=BMW3seriesParams()
        db_model = DoubleIntegrator(params=vehicle_params, delta_t=0.1)

        # check dimensions
        assert db_model.state_dimension == DIStateIndices.dim
        assert db_model.input_dimension == DIInputIndices.dim
        assert db_model.disturbance_dimension == DIDisturbanceIndices.dim

        # check input bounds
        lb, ub = db_model.input_bounds()
        assert isinstance(lb, DIInput)
        assert isinstance(ub, DIInput)
        assert lb.acceleration_long == -vehicle_params.a_long_max
        assert lb.acceleration_lat == -vehicle_params.a_lat_max
        assert ub.acceleration_long == vehicle_params.a_long_max
        assert ub.acceleration_lat == vehicle_params.a_lat_max

    def test_acceleration(self):
        """
        Tests dynamics, acceleration computation, and computation of tyre forces when accelerating while going straight.
        """

        # instantiate model
        di_model, _ = self.make_vehicle()

        # state and control input
        # ... going straight
        x = DIState(
            position_long=0.0,
            position_lat=0.0,
            velocity_long=10.0,
            velocity_lat=0.0,
        )
        # ... while accelerating
        u = DIInput(
            acceleration_long=1.0,
            acceleration_lat=1.0
        )
        # ... disturbance all zeroes
        w = DIDisturbance()
        assert np.allclose(w.convert_to_array(), np.zeros(shape=(DIDisturbanceIndices.dim,1)), 1e-12)

        # check dynamics
        f = di_model.dynamics_ct(x,u,w)

        # expected output:
        # ... derivative of long. position is long. velocity
        # ... derivative of long. velocity is acceleration
        # ... the same holds for the lateral acceleration
        f_expected = np.zeros(shape=(DIDisturbanceIndices.dim,))
        f_expected[DIStateIndices.position_long] = x.velocity_long
        f_expected[DIStateIndices.velocity_long] = u.acceleration_long
        f_expected[DIStateIndices.position_lat] = x.velocity_lat
        f_expected[DIStateIndices.velocity_lat] = u.acceleration_lat
        # check output:
        assert np.allclose(f, f_expected)

        # check acceleration
        # ... normalized longitudinal and lateral acceleration is u.acceleration_xx/vehicle_params.a_xx_max
        # ... normalized lateral acceleration is zero
        a_long_norm, a_lat_norm = di_model.compute_normalized_acceleration(x,u)
        assert np.isclose(a_long_norm, u.acceleration_long/di_model._a_long_max)
        assert np.isclose(a_long_norm, u.acceleration_lat/di_model._a_long_max)

    def test_db_sidt_factory(self):
        """
        Test factory for creating states, inputs,trajectories,etc.
        """

        # state from array
        x_rand_np = np.random.rand(DIStateIndices.dim)
        state = DISIDTFactory.state_from_numpy_array(x_rand_np)
        assert isinstance(state, DIState)
        # ... convert back to array
        assert np.isclose(state.convert_to_array(), x_rand_np).all()
        # state from args
        state = DISIDTFactory.state_from_args(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        )
        assert isinstance(state, DIState)

        # control input from array
        u_rand_np = np.random.rand(DIInputIndices.dim)
        input = DISIDTFactory.input_from_numpy_array(u_rand_np)
        assert isinstance(input, DIInput)
        # ... convert back to array
        assert np.isclose(input.convert_to_array(), u_rand_np).all()
        # control input from args
        input = DISIDTFactory.input_from_args(
            acceleration_long=1,
            acceleration_lat=2
        )
        assert isinstance(input, DIInput)

        # disturbance from array
        w_rand_np = np.random.rand(DIDisturbanceIndices.dim)
        disturbance = DISIDTFactory.disturbance_from_numpy_array(w_rand_np)
        assert isinstance(disturbance, DIDisturbance)
        # ... convert back to array
        assert np.isclose(disturbance.convert_to_array(), w_rand_np).all()
        # disturbance from args
        disturbance = DISIDTFactory.disturbance_from_args(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        )
        assert isinstance(disturbance, DIDisturbance)

        # trajectory from points
        states = {0: DIState(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        ),
                  1: DIState(
                      position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
                  )
        }
        state_traj = DISIDTFactory.trajectory_from_points(
            trajectory_dict=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, DITrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()
        # trajectory from array
        state_traj = DISIDTFactory.trajectory_from_numpy_array(
            traj_np=np.column_stack((states[0].convert_to_array(), states[1].convert_to_array())),
            mode=TrajectoryMode.State,
            time_steps=[0,1],
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, DITrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()

    def test_state_input_disturbance_noise_trajectory(self):
        """
        Test instantiation of state, input, etc. and methods for querying trajectories.
        """

        # state
        x = DIState(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        )
        # input
        u = DIInput(acceleration_long=5, acceleration_lat=5)

        # disturbance
        w = DIDisturbance(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        )
        # ... empty disturbance - default values are all zero
        w = DIDisturbance()
        assert np.isclose(w.convert_to_array(), 0).all()
        # noise (full state feedback)
        y = DINoise(
            position_long=1, position_lat=2, velocity_long=3, velocity_lat=4
        )
        # ... empty noise - default values are all zero
        y = DINoise()
        assert np.isclose(y.convert_to_array(), 0).all()

        # trajectory
        states = {0: DIState(position_long=0, position_lat=0, velocity_long=0, velocity_lat=0,
        ),
                  1: DIState(position_long=1, position_lat=1, velocity_long=1, velocity_lat=1)
        }
        state_traj = DITrajectory(
            points=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        # ... get point
        desired_state = state_traj.get_point_at_time(time=0.5, factory=DISIDTFactory())
        assert np.isclose(desired_state.convert_to_array(), 0.5*states[0].convert_to_array()+0.5*states[1].convert_to_array()).all()
        # ... convert to numpy array - no interpolation
        time = [0.25, 0.75]
        desired_states_np = state_traj.convert_to_numpy_array(time)
        assert np.isclose(desired_states_np[:,0], states[0].convert_to_array()).all()
        assert np.isclose(desired_states_np[:,1], states[1].convert_to_array()).all()
        # ... convert to numpy array - linear interpolation
        desired_states_np = state_traj.convert_to_numpy_array(time, sidt_factory=DISIDTFactory(), linear_interpolate=True)
        assert np.isclose(desired_states_np[:, 0],
                          0.75 * states[0].convert_to_array() + 0.25 * states[1].convert_to_array()).all()
        assert np.isclose(desired_states_np[:, 1],
                          0.25 * states[0].convert_to_array() + 0.75 * states[1].convert_to_array()).all()

        # ... append new final point
        new_point = DIState(
            position_long=2, position_lat=2, velocity_long=1, velocity_lat=1
        )
        state_traj.append_point(new_point)
        assert np.isclose(state_traj.final_point.convert_to_array(), new_point.convert_to_array()).all()
        assert np.isclose(state_traj.t_final, 2*state_traj.delta_t)
