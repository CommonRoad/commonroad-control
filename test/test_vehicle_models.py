import unittest

from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.simulation.simulation import Simulation

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput

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

    def test_kb_sim(self) -> None:
        # ----------------- kinematic single track ------------------

        # init vehicle model
        kb = KinematicBicycle(params=BMW3seriesParams(), delta_t=0.1)

        kb_factory: KBSITFactory = KBSITFactory()

        # init simulation
        sim = Simulation(vehicle_model=kb, state_input_factory=kb_factory)

        # set initial state and control input
        x0 = KBState(position_x=0.0, position_y=0.0, velocity=15.0, heading=0.0, steering_angle=0.0)
        u = KBInput(acceleration=0.0, steering_angle_velocity=0.15)

        # simulate
        x_sim, _, _ = sim.simulate(x0, u, time_horizon=1.0)
        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

    def test_db_sim(self) -> None:
        # ----------------- dynamic bicycle ------------------
        # init vehicle model
        db = DynamicBicycle(params=BMW3seriesParams(), delta_t=0.1)

        db_factory: DBSITFactory = DBSITFactory()

        # init simulation
        sim = Simulation(vehicle_model=db, state_input_factory=db_factory)

        # set initial state and control input
        x0 = DBState(position_x=0.0, position_y=0.0, velocity_long=15.0, velocity_lat=0.0,
                     heading=0.0, yaw_rate=0.0, steering_angle=0.0)
        u = DBInput(acceleration=0.0, steering_angle_velocity=0.15)

        # simulate
        x_sim, _, _ = sim.simulate(x0, u, time_horizon=1.0)

        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

    def test_di_sim(self) -> None:
        # ----------------- double integrator ------------------
        # init vehicle model
        di = DoubleIntegrator(params=BMW3seriesParams(), delta_t=0.1)

        di_factory = DISITFactory()

        # init simulation
        sim = Simulation(vehicle_model=di, state_input_factory=di_factory)

        # set initial state and control input
        x0 = DIState(position_long=0.0, position_lat=0.0, velocity_long=10.0, velocity_lat=0.0)
        u = DIInput(acceleration_long=0.0, acceleration_lat=0.5)

        # simulate
        x_sim, _, _ = sim.simulate(x0, u, time_horizon=1.0)
        print(f"initial state: {x0}")
        print(f"x_sim {x_sim}")

