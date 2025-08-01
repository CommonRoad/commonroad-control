import unittest

from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.simulation.simulation import Simulation

from commonroad_control.vehicle_dynamics.kinematic_single_track.kinematic_single_track import KinematicSingleStrack
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput

from commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle import DynamicBicycle
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput

from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput


class VehicleModelSimulationTest(unittest.TestCase):
    """
    Test Vehicle forward simulation
    """
    # TODO: Reasonable values to compare against

    def test_kst_sim(self) -> None:
        # ----------------- kinematic single track ------------------

        # init vehicle model
        kst = KinematicSingleStrack(params=BMW3seriesParams(), dt=0.1)

        kst_factory: KSTSITFactory = KSTSITFactory()

        # init simulation
        sim = Simulation(vehicle_model=kst, state_input_factory=kst_factory)

        # set initial state and control input
        x0 = KSTState(position_x=0.0, position_y=0.0, velocity=15.0, heading=0.0, steering_angle=0.0)
        u = KSTInput(acceleration=0.0, steering_angle_velocity=0.15)

        # simulate
        x_sim = sim.simulate(x0, u, time_horizon=1.0)
        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

    def test_dst_sim(self) -> None:
        # ----------------- dynamic bicycle ------------------
        # init vehicle model
        db = DynamicBicycle(params=BMW3seriesParams(), dt=0.1)

        dst_factory: DBSITFactory = DBSITFactory()

        # init simulation
        sim = Simulation(vehicle_model=db, state_input_factory=dst_factory)

        # set initial state and control input
        x0 = DBState(position_x=0.0, position_y=0.0, velocity_long=15.0, velocity_lat=0.0,
                     heading=0.0, yaw_rate=0.0, steering_angle=0.0)
        u = DBInput(acceleration=0.0, steering_angle_velocity=0.15)

        # simulate
        x_sim = sim.simulate(x0, u, time_horizon=1.0)

        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

    def test_di_sim(self) -> None:
        # ----------------- double integrator ------------------
        # init vehicle model
        di = DoubleIntegrator(params=BMW3seriesParams(), dt=0.1)

        di_factory = DISITFactory()

        # init simulation
        sim = Simulation(vehicle_model=di, state_input_factory=di_factory)

        # set initial state and control input
        x0 = DIState(position_long=0.0, position_lat=0.0, velocity_long=10.0, velocity_lat=0.0)
        u = DIInput(acceleration_long=0.0, acceleration_lat=0.5)

        # simulate
        x_sim = sim.simulate(x0, u, time_horizon=1.0)
        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

