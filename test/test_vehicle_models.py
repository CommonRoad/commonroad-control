import unittest

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.simulation.simulation import Simulation

from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput

from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput


class VehicleModelSimulationTest(unittest.TestCase):
    """
    Test Vehicle forward simulation
    """
    # TODO: Reasonable values to compare against


    def test_kst_sim(self) -> None:
        # ----------------- kinematic single track ------------------
        # init vehicle model
        kst = KinematicSingleStrack(params=BMW3seriesParams(), dt=0.1)

        # init simulation
        sim = Simulation(kst)

        # set initial state and control input
        x0 = KSTState(position_x=0.0, position_y=0.0, velocity=10.0, acceleration=0.0, heading=0.0, steering_angle=0.1)
        u = KSTInput(jerk=0.0, steering_angle_velocity=0.0)

        # simulate
        x_sim = sim.simulate(x0, u, time_horizon=1.0)
        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")


    def test_dst_sim(self) -> None:
        # ----------------- dynamic bicycle ------------------
        # init vehicle model
        db = DynamicBicycle(params=BMW3seriesParams(), dt=0.1)

        # init simulation
        sim = Simulation(db)

        # set initial state and control input
        x0 = DBState(position_x=0.0, position_y=0.0, velocity_long=10.0, velocity_lat=0.0,
                     acceleration=0.0, heading=0.0, yaw_rate=0.0, steering_angle=0.0)
        u = DBInput(jerk=0.0, steering_angle_velocity=0.0)

        # simulate
        x_sim = sim.simulate(x0, u, time_horizon=1.0)

        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

