import unittest

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory


class TestModelInstantiation(unittest.TestCase):
    """
    Tests simple instantiation
    """

    def test_kst_inst(self) -> None:
        """
        Test Kinematic Single Track state, input and trajectory
        """
        kst_model = KinematicSingleStrack(params=BMW3seriesParams(), dt=0.1)

        states = {0: KSTState(position_x=5, position_y=5, velocity=5, heading=5, steering_angle=5),
                  1: KSTState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
        state_traj = KSTTrajectory(
            mode='state',
            points=states,
            t_0=0,
            delta_t=0.5
        )

        inputs = {0: KSTInput(acceleration=5, steering_angle_velocity=5), 1: KSTInput(acceleration=3, steering_angle_velocity=3)}
        input_traj = KSTTrajectory(
            mode='input',
            points=inputs,
            t_0=0,
            delta_t=0.5
        )

    def test_dst_inst(self) -> None:
        """
        Test Dynamic single track
        """
        dst_model = DynamicBicycle(params=BMW3seriesParams(), dt=0.1)

        states = {0: DBState(
            position_x=5, position_y=5, velocity_long=5, velocity_lat=5, heading=5, steering_angle=5
        ),
                  1: DBState(
          position_x=3, position_y=3, velocity_long=3, velocity_lat=3, heading=3, steering_angle=3
        )}

        state_traj = DBTrajectory(
            mode='state',
            points=states,
            t_0=0,
            delta_t=0.5
        )

        inputs = {0: DBInput(acceleration=5, steering_angle_velocity=5), 1: DBInput(acceleration=3, steering_angle_velocity=3)}
        input_traj = DBTrajectory(
            mode='input',
            points=inputs,
            t_0=0,
            delta_t=0.5
        )

    def test_di_inst(self) -> None:
        # TODO: add double integrator tests
        pass
