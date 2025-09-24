import unittest

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory


class TestModelInstantiation(unittest.TestCase):
    """
    Tests simple instantiation
    """


    def test_kb_inst(self) -> None:
       """
       Test Kinematic Bicycle state, input, and trajectory
       """
       kst_model = KinematicBicycle(params=BMW3seriesParams(), delta_t=0.1)

       states = {0: KBState(position_x=5, position_y=5, velocity=5, heading=5, steering_angle=5),
                 1: KBState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
       state_traj = KBTrajectory(
           mode=TrajectoryMode.State,
           points=states,
           t_0=0,
           delta_t=0.5
       )

       inputs = {0: KBInput(acceleration=5, steering_angle_velocity=5), 1: KBInput(acceleration=3, steering_angle_velocity=3)}
       input_traj = KBTrajectory(
           mode=TrajectoryMode.Input,
           points=inputs,
           t_0=0,
           delta_t=0.5
       )

    def test_db_inst(self) -> None:
        """
        Test Dynamic bicycle state, input, and trajectory
        """
        db_model = DynamicBicycle(params=BMW3seriesParams(), delta_t=0.1)

        states = {0: DBState(
            position_x=5, position_y=5, velocity_long=5, velocity_lat=5, heading=5, steering_angle=5
        ),
                  1: DBState(
          position_x=3, position_y=3, velocity_long=3, velocity_lat=3, heading=3, steering_angle=3
        )}

        state_traj = DBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.5
        )

        inputs = {0: DBInput(acceleration=5, steering_angle_velocity=5), 1: DBInput(acceleration=3, steering_angle_velocity=3)}
        input_traj = DBTrajectory(
            mode=TrajectoryMode.Input,
            points=inputs,
            t_0=0,
            delta_t=0.5
        )

    def test_di_inst(self) -> None:
        # TODO: add double integrator tests
        pass
