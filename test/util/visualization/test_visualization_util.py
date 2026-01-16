import unittest
import numpy as np

from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput

from commonroad_control.vehicle_dynamics.double_integrator.di_trajectory import DITrajectory

from commonroad_control.util.visualization.visualization_util import time_synchronize_trajectories

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import DISIDTFactory


class TestVisualizationUtil(unittest.TestCase):
    """
    Tests for time-synchronization of trajectories (visualization utilities).
    """


    def test_time_synchronize_trajectories(self) -> None:
        """
        Test for time-synchronizing two trajectories for plotting.
        Test case: asynchronous trajectory is sampled at a higher rate, no clamping.
        Reference trajectory: delta_t = 0.75  -> t = [0, 0.75, 1.5]
        Asynchronous trajectory: delta_t = 0.5  -> t = [0, 0.5, 1.0, 1.5]
        """

        # reference trajectory (lower rate)
        ref_points = {
            0: DIInput(0.0, 0.0),
            1: DIInput(0.0, 0.0),
            2: DIInput(0.0, 0.0),
        }

        reference_trajectory = DITrajectory(
            points=ref_points,
            delta_t=0.75,
            mode=TrajectoryMode.Input,
            t_0=0.0,
        )

        # asynchronous trajectory (higher rate)
        async_points = {
            0: DIInput(0.0, 0.0),
            1: DIInput(0.5, 1.0),
            2: DIInput(1.0, 2.0),
            3: DIInput(1.5, 3.0),
        }
        async_trajectory = DITrajectory(
            points=async_points,
            delta_t=0.5,
            mode=TrajectoryMode.Input,
            t_0=0.0,
        )

        # time-synchronize trajectories
        resampled_trajectory = time_synchronize_trajectories(
            reference_trajectory=reference_trajectory,
            asynchronous_trajectory=async_trajectory,
            sidt_factory=DISIDTFactory()
        )

        # check results
        assert np.isclose(reference_trajectory.t_0, resampled_trajectory.t_0)
        assert np.isclose(reference_trajectory.delta_t, resampled_trajectory.delta_t)
        assert np.isclose(reference_trajectory.t_final, resampled_trajectory.t_final)

        assert np.allclose(resampled_trajectory.points[0].convert_to_array(), async_points[0].convert_to_array())
        assert np.allclose(resampled_trajectory.points[1].convert_to_array(), np.array([0.75, 1.5]))
        assert np.allclose(resampled_trajectory.points[2].convert_to_array(), async_points[3].convert_to_array())

    def test_time_synchronization_reference_higher_sampling_rate(self) -> None:
        """
        Test for time-synchronizing two trajectories for plotting.
        Test case: reference trajectory is sampled at a higher rate, no clamping.
        Reference trajectory: delta_t = 0.25 → t = [0, 0.25, 0.5, ..., 2.0]
        Asynchronous trajectory: delta_t = 1.0 → t = [0, 1, 2]
       """

        # reference trajectory (higher rate)
        ref_points = {
            i: DIInput(0.0, 0.0) for i in range(9)  # steps 0..8 → t = 0..2.0
        }
        reference_trajectory = DITrajectory(
            points=ref_points,
            delta_t=0.25,
            mode=TrajectoryMode.Input,
            t_0=0.0,
        )

        # asynchronous trajectory (lower rate)
        async_points = {
            0: DIInput(0.0, 0.0),
            1: DIInput(1.0, 2.0),
            2: DIInput(2.0, 4.0),
        }
        async_trajectory = DITrajectory(
            points=async_points,
            delta_t=1.0,
            mode=TrajectoryMode.Input,
            t_0=0.0,
        )

        # time-synchronize trajectories
        resampled_trajectory = time_synchronize_trajectories(
            reference_trajectory=reference_trajectory,
            asynchronous_trajectory=async_trajectory,
            sidt_factory=DISIDTFactory()
        )

        # check results
        assert np.isclose(reference_trajectory.t_0, resampled_trajectory.t_0)
        assert np.isclose(reference_trajectory.delta_t, resampled_trajectory.delta_t)
        assert np.isclose(reference_trajectory.t_final, resampled_trajectory.t_final)
        # ... check interpolation at t = 0.5
        step = 2
        assert np.allclose(resampled_trajectory.points[step].convert_to_array(), np.array([0.5, 1.0]))
        # ... check interpolation at t = 1.75
        step = 7
        assert np.allclose(resampled_trajectory.points[step].convert_to_array(), np.array([1.75, 3.5]))

    def test_time_synchronization_with_clamping(self) -> None:
        """
        Reference trajectory samples outside asynchronous trajectory time range.

        Reference trajectory: t ∈ [0.0, 2.0]
        Async trajectory: t ∈ [0.5, 1.5]
        """

        # reference trajectory (higher rate)
        ref_points = {
            i: DIInput(0.0, 0.0) for i in range(9)  # steps 0..8 → t = 0..2.0
        }
        reference_trajectory = DITrajectory(
            points=ref_points,
            delta_t=0.25,
            mode=TrajectoryMode.Input,
            t_0=0.0,
        )

        # asynchronous trajectory (lower rate)
        async_points = {
            0: DIInput(0.0, 0.0),
            1: DIInput(1.0, 2.0),
            2: DIInput(2.0, 4.0),
        }
        async_trajectory = DITrajectory(
            points=async_points,
            delta_t=0.5,
            mode=TrajectoryMode.Input,
            t_0=0.5,
        )

        # time-synchronize trajectories
        resampled_trajectory = time_synchronize_trajectories(
            reference_trajectory=reference_trajectory,
            asynchronous_trajectory=async_trajectory,
            sidt_factory=DISIDTFactory()
        )

        # check results
        assert np.isclose(reference_trajectory.t_0, resampled_trajectory.t_0)
        assert np.isclose(reference_trajectory.delta_t, resampled_trajectory.delta_t)
        assert np.isclose(reference_trajectory.t_final, resampled_trajectory.t_final)
        # .... t <= 0.5 -> clamped to initial point of asynchronous trajectory
        time = 0.0
        assert np.allclose(resampled_trajectory.get_point_at_time(time=time, sidt_factory=DISIDTFactory()).convert_to_array(),
                           async_trajectory.initial_point.convert_to_array())
        time = 0.5
        assert np.allclose(resampled_trajectory.get_point_at_time(time=time, sidt_factory=DISIDTFactory()).convert_to_array(),
                           async_trajectory.initial_point.convert_to_array())
        # ... check interpolation at t = 1.25s
        time = 1.25
        assert np.allclose(resampled_trajectory.get_point_at_time(time=time, sidt_factory=DISIDTFactory()).convert_to_array(), np.array([1.5, 3.0]))
        # .... t >= 1.5s -> clamped to final point of asynchronous trajectory
        time = 1.5
        assert np.allclose(resampled_trajectory.get_point_at_time(time=time, sidt_factory=DISIDTFactory()).convert_to_array(),
                           async_trajectory.final_point.convert_to_array())
        time = 2.0
        assert np.allclose(resampled_trajectory.get_point_at_time(time=time, sidt_factory=DISIDTFactory()).convert_to_array(),
                           async_trajectory.final_point.convert_to_array())

