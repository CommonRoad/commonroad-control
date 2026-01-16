import numpy as np
import unittest
from typing import Tuple
import logging
from math import cos, sin

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState, DBStateIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput,DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_disturbance import (
    DBDisturbance,DBDisturbanceIndices
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_noise import DBNoise
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory


logger_global = configure_toolbox_logging(level=logging.DEBUG)


class TestDynamicBicycle(unittest.TestCase):
    """
    Tests for dynamic bicycle model:
        - vehicle model:
            - instantiation
            - tyre forces
            - dynamics
            - normalized acceleration
        - sidt_factory
        - (querying) trajectory
    """

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

    def test_db_instanation(self) -> None:
        """
        Test correct instantiation of model.
        """

        vehicle_params=BMW3seriesParams()
        db_model = DynamicBicycle(params=vehicle_params, delta_t=0.1)

        # check dimensions
        assert db_model.state_dimension == DBStateIndices.dim
        assert db_model.input_dimension == DBInputIndices.dim
        assert db_model.disturbance_dimension == DBDisturbanceIndices.dim

        # check input bounds
        lb, ub = db_model.input_bounds()
        assert isinstance(lb, DBInput)
        assert isinstance(ub, DBInput)
        assert lb.acceleration == -vehicle_params.a_long_max
        assert lb.steering_angle_velocity == -vehicle_params.steering_angle_velocity_max
        assert ub.acceleration == vehicle_params.a_long_max
        assert ub.steering_angle_velocity == vehicle_params.steering_angle_velocity_max

    def test_acceleration(self):
        """
        Tests dynamics, acceleration computation, and computation of tyre forces when accelerating while going straight.
        """

        # instantiate model
        db_model, _ = self.make_vehicle()

        # state and control input
        # ... going straight
        x = DBState(
            position_x=0.0,
            position_y=0.0,
            velocity_long=10.0,
            velocity_lat=0.0,
            heading=0.0,
            yaw_rate=0.0,
            steering_angle=0.0
        )
        # ... while accelerating
        u = DBInput(
            acceleration=1.0,
            steering_angle_velocity=0.0
        )
        # ... disturbance all zeroes
        w = DBDisturbance()
        assert np.allclose(w.convert_to_array(), np.zeros(shape=(DBDisturbanceIndices.dim,1)), 1e-12)

        # check dynamics
        f = db_model.dynamics_ct(x,u,w)

        # expected output:
        # ... derivative of long. position is long. velocity
        # ... derivative of velocity is acceleration
        f_expected = np.zeros(shape=(DBDisturbanceIndices.dim,))
        f_expected[DBStateIndices.position_x] = x.velocity_long
        f_expected[DBStateIndices.velocity_long] = u.acceleration
        # check output:
        assert np.allclose(f, f_expected)

        # check acceleration
        # ... normalized longitudinal acceleration is u.acceleration/vehicle_params.a_long_max
        # ... normalized lateral acceleration is zero
        a_long_norm, a_lat_norm = db_model.compute_normalized_acceleration(x,u)
        assert np.isclose(a_long_norm, u.acceleration/db_model._a_long_max)
        assert np.isclose(a_lat_norm, 0)

        # check lateral tyre forces
        # ... both must be zero
        fc_f, fc_r = db_model._compute_lateral_tyre_forces(x.convert_to_array(), u.convert_to_array())
        assert np.isclose(fc_f, 0.0)
        assert np.isclose(fc_r, 0.0)

    def test_steady_state_cornering(self):
        """
        Tests dynamics, acceleration computation, and computation of tyre forces for steady-state cornering.
        """

        # instantiate model
        db_model, _ = self.make_vehicle()

        # steady-state cornering
        # ... fix long. velocity and yaw rate
        v_long = 15.0  # [m/s] longitudinal velocity
        yaw_rate = 0.15  # [rad/s] yaw rate -> ~ medium turn
        # ... normal load distribution (no pitch, no long. acceleration)
        f_zf = db_model._m*db_model._g*db_model._l_r/db_model._l_wb
        f_zr = db_model._m*db_model._g - f_zf
        # ... (yaw) torque balance around center of gravity (small angle approximation) and lateral force balance
        f_yf = db_model._m*v_long*yaw_rate*db_model._l_r/db_model._l_wb
        f_yr = f_yf*db_model._l_f/db_model._l_r
        # ... compute slip angles
        alpha_f = -f_yf/(f_zf*db_model._C_f)
        alpha_r = -f_yr/(f_zr*db_model._C_r)
        # ... compute steering angle (subtract slip angle definitions + small-angle approximation)
        delta_ss = db_model._l_wb*yaw_rate/v_long - (alpha_f-alpha_r)
        # ... lateral velocity (see pervious step)
        v_lat = v_long*alpha_r + db_model._l_r*yaw_rate

        # initialize state
        x = DBState(
            position_x=0.0,
            position_y=0.0,
            velocity_long=v_long,
            velocity_lat=v_lat,
            heading=0.0,
            yaw_rate=yaw_rate,
            steering_angle=delta_ss
        )
        # initialize input: no acceleration, constant steering angle
        u = DBInput(
            acceleration=0.0,
            steering_angle_velocity=0.0
        )

        # ... disturbance all zeroes
        w = DBDisturbance()

        # check dynamics
        f = db_model.dynamics_ct(x,u,w)
        # ... yaw rate derivative almost zero (due to approximations made above)
        assert np.isclose(f[DBStateIndices.yaw_rate],0, atol=1e-3)
        # ... lateral velocity derivative almost zero (due to approximations made above)
        assert np.isclose(f[DBStateIndices.velocity_lat], 0, atol=1e-3)
        # ... vehicle rotates counter-clockwise -> positive derivative of lateral position
        assert float(f[DBStateIndices.position_y]) > 0

        # Compute lateral tire forces and normalized accelerations
        fc_f, fc_r = db_model._compute_lateral_tyre_forces(x.convert_to_array(), u.convert_to_array())
        a_long_norm, a_lat_norm = db_model.compute_normalized_acceleration(x, u)
        # ... unscale accelerations
        a_long = a_long_norm * db_model._a_long_max
        a_lat = a_lat_norm * db_model._a_lat_max
        # ... yaw moment balance
        f_lat_total = float(fc_f*cos(x.steering_angle) + fc_r)
        assert np.isclose((fc_f*cos(x.steering_angle)*db_model._l_f - fc_r*db_model._l_r) / (0.5*f_lat_total), 0, atol=1e-3)
        # .... lateral acceleration ~ v_x*yaw_rate
        assert abs(float(a_lat) - v_long*yaw_rate) < 1e-3
        # .... longitudinal acceleration ~ drag due to steered front axle
        assert abs(float(a_long) + float(fc_f)*sin(x.steering_angle)/db_model._m) < 1e-3

    def test_db_sidt_factory(self):
        """
        Test factory for creating states, inputs,trajectories,etc.
        """

        # state from array
        x_rand_np = np.random.rand(DBStateIndices.dim)
        state = DBSIDTFactory.state_from_numpy_array(x_rand_np)
        assert isinstance(state, DBState)
        # ... convert back to array
        assert np.isclose(state.convert_to_array(), x_rand_np).all()
        # state from args
        state = DBSIDTFactory.state_from_args(position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        )
        assert isinstance(state, DBState)

        # control input from array
        u_rand_np = np.random.rand(DBInputIndices.dim)
        input = DBSIDTFactory.input_from_numpy_array(u_rand_np)
        assert isinstance(input, DBInput)
        # ... convert back to array
        assert np.isclose(input.convert_to_array(), u_rand_np).all()
        # control input from args
        input = DBSIDTFactory.input_from_args(
            acceleration=1,
            steering_angle_velocity=2
        )
        assert isinstance(input, DBInput)

        # disturbance from array
        w_rand_np = np.random.rand(DBDisturbanceIndices.dim)
        disturbance = DBSIDTFactory.disturbance_from_numpy_array(w_rand_np)
        assert isinstance(disturbance, DBDisturbance)
        # ... convert back to array
        assert np.isclose(disturbance.convert_to_array(), w_rand_np).all()
        # disturbance from args
        disturbance = DBSIDTFactory.disturbance_from_args(position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        )
        assert isinstance(disturbance, DBDisturbance)

        # trajectory from points
        states = {0: DBState(position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        ),
                  1: DBState(position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6,yaw_rate=7)
        }
        state_traj = DBSIDTFactory.trajectory_from_points(
            trajectory_dict=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, DBTrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()
        # trajectory from array
        state_traj = DBSIDTFactory.trajectory_from_numpy_array(
            traj_np=np.column_stack((states[0].convert_to_array(), states[1].convert_to_array())),
            mode=TrajectoryMode.State,
            time_steps=[0,1],
            t_0=0.0,
            delta_t=1.0
        )
        assert isinstance(state_traj, DBTrajectory)
        # ... compare initial and final points
        assert np.isclose(states[0].convert_to_array(), state_traj.initial_point.convert_to_array()).all()
        assert np.isclose(states[1].convert_to_array(), state_traj.final_point.convert_to_array()).all()

    def test_state_input_disturbance_noise_trajectory(self):
        """
        Test instantiation of state, input, etc. and methods for querying trajectories.
        """

        # state
        x = DBState(
            position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        )
        # input
        u = DBInput(acceleration=5, steering_angle_velocity=5)

        # disturbance
        w = DBDisturbance(
            position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        )
        # ... empty disturbance - default values are all zero
        w = DBDisturbance()
        assert np.isclose(w.convert_to_array(), 0).all()
        # noise (full state feedback)
        y = DBNoise(
            position_x=1, position_y=2, velocity_long=3, velocity_lat=4, heading=5, steering_angle=6, yaw_rate=7
        )
        # ... empty noise - default values are all zero
        y = DBNoise()
        assert np.isclose(y.convert_to_array(), 0).all()

        # trajectory
        states = {0: DBState(position_x=0, position_y=0, velocity_long=0, velocity_lat=0, heading=0, steering_angle=0, yaw_rate=0
        ),
                  1: DBState(position_x=1, position_y=1, velocity_long=1, velocity_lat=1, heading=1, steering_angle=1,yaw_rate=1)
        }
        state_traj = DBTrajectory(
            points=states,
            mode=TrajectoryMode.State,
            t_0=0.0,
            delta_t=1.0
        )
        # ... get point
        desired_state = state_traj.get_point_at_time(time=0.5, sidt_factory=DBSIDTFactory())
        assert np.isclose(desired_state.convert_to_array(), 0.5*states[0].convert_to_array()+0.5*states[1].convert_to_array()).all()
        # ... convert to numpy array - no interpolation
        time = [0.25, 0.75]
        desired_states_np = state_traj.convert_to_numpy_array(time)
        assert np.isclose(desired_states_np[:,0], states[0].convert_to_array()).all()
        assert np.isclose(desired_states_np[:,1], states[1].convert_to_array()).all()
        # ... convert to numpy array - linear interpolation
        desired_states_np = state_traj.convert_to_numpy_array(time, sidt_factory=DBSIDTFactory(), linear_interpolate=True)
        assert np.isclose(desired_states_np[:, 0],
                          0.75 * states[0].convert_to_array() + 0.25 * states[1].convert_to_array()).all()
        assert np.isclose(desired_states_np[:, 1],
                          0.25 * states[0].convert_to_array() + 0.75 * states[1].convert_to_array()).all()

        # ... append new final point
        new_point = DBState(
            position_x=2, position_y=2, velocity_long=1, velocity_lat=1, heading=1, steering_angle=1,yaw_rate=1
        )
        state_traj.append_point(new_point)
        assert np.isclose(state_traj.final_point.convert_to_array(), new_point.convert_to_array()).all()
        assert np.isclose(state_traj.t_final, 2*state_traj.delta_t)
