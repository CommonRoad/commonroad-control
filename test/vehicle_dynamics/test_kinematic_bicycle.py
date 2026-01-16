import numpy as np
import unittest
from typing import Tuple
import logging
from math import cos, sin, asin, tan, atan

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState, KBStateIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput,KBInputIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_disturbance import (
    KBDisturbance,KBDisturbanceIndices
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_noise import KBNoise
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import KBSIDTFactory


logger_global = configure_toolbox_logging(level=logging.DEBUG)


class TestKinematicBicycle(unittest.TestCase):
    """
    Tests for kinematic bicycle model:
        - vehicle model:
            - instantiation
            - dynamics
            - normalized acceleration
        - sidt_factory
        - (querying) trajectory
    """

    @staticmethod
    def make_vehicle(
    ) -> Tuple[KinematicBicycle, KBSIDTFactory]:
        # parameters for simulation
        vehicle_params = BMW3seriesParams()
        dt = 0.1

        # setup simulation
        # ... vehicle model
        sit_factory_sim = KBSIDTFactory()
        vehicle_model_sim = KinematicBicycle(params=vehicle_params, delta_t=dt)

        return vehicle_model_sim, sit_factory_sim

    def test_kb_instanation(self) -> None:
        """
        Test correct instantiation of model.
        """
        vehicle_params=BMW3seriesParams()
        db_model = KinematicBicycle(params=vehicle_params, delta_t=0.1)

        # check dimensions
        assert db_model.state_dimension == KBStateIndices.dim
        assert db_model.input_dimension == KBInputIndices.dim
        assert db_model.disturbance_dimension == KBDisturbanceIndices.dim

        # check input bounds
        lb, ub = db_model.input_bounds()
        assert isinstance(lb, KBInput)
        assert isinstance(ub, KBInput)
        assert lb.acceleration == -vehicle_params.a_long_max
        assert lb.steering_angle_velocity == -vehicle_params.steering_angle_velocity_max
        assert ub.acceleration == vehicle_params.a_long_max
        assert ub.steering_angle_velocity == vehicle_params.steering_angle_velocity_max

    def test_acceleration(self):
        """
        Tests dynamics, acceleration computation, and computation of tyre forces when accelerating while going straight.
        """

        # instantiate model
        kb_model, _ = self.make_vehicle()

        # state and control input
        # ... going straight
        x = KBState(
            position_x=0.0,
            position_y=0.0,
            velocity=10.0,
            heading=0.0,
            steering_angle=0.0
        )
        # ... while accelerating
        u = KBInput(
            acceleration=1.0,
            steering_angle_velocity=0.0
        )
        # ... disturbance all zeroes
        w = KBDisturbance()
        assert np.allclose(w.convert_to_array(), np.zeros(shape=(KBDisturbanceIndices.dim,1)), 1e-12)

        # check dynamics
        f = kb_model.dynamics_ct(x,u,w)

        # expected output:
        # ... derivative of long. position is long. velocity
        # ... derivative of velocity is acceleration
        f_expected = np.zeros(shape=(KBDisturbanceIndices.dim,))
        f_expected[KBStateIndices.position_x] = x.velocity
        f_expected[KBStateIndices.velocity] = u.acceleration
        # check output:
        assert np.allclose(f, f_expected)

        # check acceleration
        # ... normalized longitudinal acceleration is u.acceleration/vehicle_params.a_long_max
        # ... normalized lateral acceleration is zero
        a_long_norm, a_lat_norm = kb_model.compute_normalized_acceleration(x,u)
        assert np.isclose(a_long_norm, u.acceleration/kb_model._a_long_max)
        assert np.isclose(a_lat_norm, 0)

    def test_steady_state_cornering(self):
        """
        Tests dynamics, acceleration computation, and computation of tyre forces for steady-state cornering.
        """

        # instantiate model
        kb_model, _ = self.make_vehicle()

        # steady-state cornering
        # ... fix long. velocity and yaw rate
        v_long = 15.0  # [m/s] longitudinal velocity
        yaw_rate = 0.15  # [rad/s] yaw rate -> ~ medium turn
        # ... compute steering angle (subtract slip angle definitions + small-angle approximation)
        slip_angle = asin(yaw_rate*kb_model._l_r/v_long)
        delta_ss = atan(tan(slip_angle)*kb_model._l_wb/kb_model._l_r)

        # initialize state
        x = KBState(
            position_x=0.0,
            position_y=0.0,
            velocity=v_long,
            heading=0.0,
            steering_angle=delta_ss
        )
        # initialize input: no acceleration, constant steering angle
        u = KBInput(
            acceleration=0.0,
            steering_angle_velocity=0.0
        )

        # ... disturbance all zeroes
        w = KBDisturbance()

        # check dynamics
        f = kb_model.dynamics_ct(x,u,w)
        # ... headin derivative = yaw rate
        assert np.isclose(f[KBStateIndices.heading],yaw_rate)
        # ... vehicle rotates counter-clockwise -> positive derivative of lateral position
        assert float(f[KBStateIndices.position_y]) > 0

        # Compute normalized accelerations
        a_long_norm, a_lat_norm = kb_model.compute_normalized_acceleration(x, u)
        # ... unscale accelerations
        a_long = a_long_norm * kb_model._a_long_max
        a_lat = a_lat_norm * kb_model._a_lat_max
        # ... yaw moment balance
        # .... lateral acceleration = v_x*yaw_rate
        assert np.isclose(a_lat,v_long*yaw_rate)

    def test_db_sidt_factory(self):
        """
        Test factory for creating states, inputs,trajectories,etc.
        """

        # state from array
        x_rand_np = np.random.rand(KBStateIndices.dim)
        state = KBSIDTFactory.state_from_numpy_array(x_rand_np)
        assert isinstance(state, KBState)
        # ... convert back to array
        assert np.isclose(state.convert_to_array(), x_rand_np).all()
        # state from args
        state = KBSIDTFactory.state_from_args(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        )
        assert isinstance(state, KBState)

        # control input from array
        u_rand_np = np.random.rand(KBInputIndices.dim)
        input = KBSIDTFactory.input_from_numpy_array(u_rand_np)
        assert isinstance(input, KBInput)
        # ... convert back to array
        assert np.isclose(input.convert_to_array(), u_rand_np).all()
        # control input from args
        input = KBSIDTFactory.input_from_args(
            acceleration=1,
            steering_angle_velocity=2
        )
        assert isinstance(input, KBInput)

        # disturbance from array
        w_rand_np = np.random.rand(KBDisturbanceIndices.dim)
        disturbance = KBSIDTFactory.disturbance_from_numpy_array(w_rand_np)
        assert isinstance(disturbance, KBDisturbance)
        # ... convert back to array
        assert np.isclose(disturbance.convert_to_array(), w_rand_np).all()
        # disturbance from args
        disturbance = KBSIDTFactory.disturbance_from_args(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        )
        assert isinstance(disturbance, KBDisturbance)

        # trajectory from points
        states = {0: KBState(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        ),
                  1: KBState(
                      position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
                  )
        }
        state_traj = KBSIDTFactory.trajectory_from_points(
            trajectory_dict=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, KBTrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()
        # trajectory from array
        state_traj = KBSIDTFactory.trajectory_from_numpy_array(
            traj_np=np.column_stack((states[0].convert_to_array(), states[1].convert_to_array())),
            mode=TrajectoryMode.State,
            time_steps=[0,1],
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, KBTrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()

    def test_state_input_disturbance_noise_trajectory(self):
        """
        Test instantiation of state, input, etc. and methods for querying trajectories.
        """

        # state
        x = KBState(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        )
        # input
        u = KBInput(acceleration=5, steering_angle_velocity=5)

        # disturbance
        w = KBDisturbance(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        )
        # ... empty disturbance - default values are all zero
        w = KBDisturbance()
        assert np.isclose(w.convert_to_array(), 0).all()
        # noise (full state feedback)
        y = KBNoise(
            position_x=1, position_y=2, velocity=3, heading=4, steering_angle=5
        )
        # ... empty noise - default values are all zero
        y = KBNoise()
        assert np.isclose(y.convert_to_array(), 0).all()

        # trajectory
        states = {0: KBState(position_x=0, position_y=0, velocity=0, heading=0, steering_angle=0,
        ),
                  1: KBState(position_x=1, position_y=1, velocity=1, heading=1, steering_angle=1)
        }
        state_traj = KBTrajectory(
            points=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        # ... get point
        desired_state = state_traj.get_point_at_time(time=0.5, sidt_factory=KBSIDTFactory())
        assert np.isclose(desired_state.convert_to_array(), 0.5*states[0].convert_to_array()+0.5*states[1].convert_to_array()).all()
        # ... convert to numpy array - no interpolation
        time = [0.25, 0.75]
        desired_states_np = state_traj.convert_to_numpy_array(time)
        assert np.isclose(desired_states_np[:,0], states[0].convert_to_array()).all()
        assert np.isclose(desired_states_np[:,1], states[1].convert_to_array()).all()
        # ... convert to numpy array - linear interpolation
        desired_states_np = state_traj.convert_to_numpy_array(time, sidt_factory=KBSIDTFactory(), linear_interpolate=True)
        assert np.isclose(desired_states_np[:, 0],
                          0.75 * states[0].convert_to_array() + 0.25 * states[1].convert_to_array()).all()
        assert np.isclose(desired_states_np[:, 1],
                          0.25 * states[0].convert_to_array() + 0.75 * states[1].convert_to_array()).all()

        # ... append new final point
        new_point = KBState(
            position_x=2, position_y=2, velocity=1, heading=1, steering_angle=1,
        )
        state_traj.append_point(new_point)
        assert np.isclose(state_traj.final_point.convert_to_array(), new_point.convert_to_array()).all()
        assert np.isclose(state_traj.t_final, 2*state_traj.delta_t)
