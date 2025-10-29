import unittest
from unittest.mock import patch
import warnings
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Agg")  # prevent GUI windows

# function under test
from commonroad_control.util.visualization.visualize_control_state import visualize_reference_vs_actual_states

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState
from commonroad_control.vehicle_dynamics.double_integrator.di_trajectory import DITrajectory

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory

class TestVisualizeControlState(unittest.TestCase):
    """
    Tests visualization of state trajectories and tracking errors.
    """

    @patch("matplotlib.pyplot.subplots",wraps=plt.subplots)
    def test_visualize_state_no_names_given(self, mock_subplots) -> None:
        """
        First, we only pass the trajectories as well as the time steps.
        We use the kinematic and dynamic bicycle model.
        """

        # dummy reference trajectory
        states = {0: DBState(
            position_x=5, position_y=5, velocity_long=5, velocity_lat=5, heading=5, steering_angle=5
        ),
                  1: DBState(
          position_x=3, position_y=3, velocity_long=3, velocity_lat=3, heading=3, steering_angle=3
        )}

        ref_db = DBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.5
        )

        # dummy actual trajectory
        states = {0: KBState(position_x=5, position_y=5, velocity=5, heading=5, steering_angle=5),
                  1: KBState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
        actual_kb = KBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.5
        )

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=UserWarning)
            visualize_reference_vs_actual_states(
                reference_trajectory=ref_db,
                actual_trajectory=actual_kb,
                time_steps=ref_db.steps,
            )

        # Expect subplots to be called twice - once for the plot of the trajectories, once for the error
        self.assertEqual(mock_subplots.call_count, 2)


    @patch("matplotlib.pyplot.subplots",wraps=plt.subplots)
    def test_visualize_state_names_given(self, mock_subplots) -> None:
        """
        Now we provide the state components for plotting.
        We use the kinematic and dynamic bicycle model and plot the velocity and the heading - these attributes are
        defined for both StateInterfaces.
        """

        # dummy reference trajectory
        states = {0: DBState(
            position_x=5, position_y=5, velocity_long=5, velocity_lat=5, heading=5, steering_angle=5
        ),
                  1: DBState(
          position_x=3, position_y=3, velocity_long=3, velocity_lat=3, heading=3, steering_angle=3
        )}

        ref_db = DBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.5
        )

        # dummy actual trajectory
        states = {0: KBState(position_x=5, position_y=5, velocity=5, heading=5, steering_angle=5),
                  1: KBState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
        actual_kb = KBTrajectory(
            mode=TrajectoryMode.State,
            points=states,
            t_0=0,
            delta_t=0.5
        )

        visualize_reference_vs_actual_states(
            reference_trajectory=ref_db,
            actual_trajectory=actual_kb,
            time_steps=ref_db.steps,
            state_names=['heading','velocity']
        )

        # Expect subplots to be called twice - once for the plot of the trajectories, once for the error
        self.assertEqual(mock_subplots.call_count, 2)



    @patch("matplotlib.pyplot.subplots", wraps=plt.subplots)
    def test_visualize_no_plot(self, mock_subplots) -> None:
        """
        Now we try to plot trajectories from the double integrator and the kinematic bicycle model - since the
        respective StateInterface objects have no common attributes, no plot is created.
        """

        # dummy reference trajectory
        states = {0: DIState(
            position_long=5, position_lat=5, velocity_long=5, velocity_lat=5
        ),
                  1: DIState(
          position_long=3, position_lat=3, velocity_long=3, velocity_lat=3
        )}
        ref_di = DITrajectory(
             mode=TrajectoryMode.State,
             points=states,
             t_0=0,
             delta_t=0.5
        )

        # dummy actual trajectory
        states = {0: KBState(position_x=5, position_y=5, velocity=5, heading=5, steering_angle=5),
                   1: KBState(position_x=3, position_y=3, velocity=5, heading=3, steering_angle=3)}
        actual_kb = KBTrajectory(
             mode=TrajectoryMode.State,
             points=states,
             t_0=0,
             delta_t=0.5
        )

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=UserWarning)
            visualize_reference_vs_actual_states(
                reference_trajectory=ref_di,
                actual_trajectory=actual_kb,
                time_steps=ref_di.steps,
            )

        # Expect subplots to be called twice - once for the plot of the trajectories, once for the error
        self.assertEqual(mock_subplots.call_count, 0)

