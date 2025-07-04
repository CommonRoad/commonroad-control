from typing import Union, Any

from scipy.integrate import solve_ivp

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class Simulation:
    def __init__(
            self,
            vehicle_model: VehicleModelInterface,
            state_input_factory: KSTSITFactory
    ):

        self._vehicle_model: VehicleModelInterface = vehicle_model
        self._state_input_factory: KSTSITFactory = state_input_factory

    def simulate(self, x0: StateInterface, u: InputInterface, time_horizon: float) -> Union[StateInterface, Any]:

        x0_num = x0.convert_to_array()
        u_num = u.convert_to_array()

        # TODO fix to public var
        res_sim = solve_ivp(lambda t, y: self._vehicle_model._dynamics_ct(y, u_num), [0, time_horizon], y0=x0_num,
                          method='RK45')
        x_end = res_sim.y[:, -1]

        # TODO: convert to state interface
        x_forward = self._state_input_factory.state_from_args(
            position_x=x_end[0],
            position_y=x_end[1],
            velocity=x_end[2],
            heading=x_end[3],
            steering_angle=x_end[4]
        )


        return x_forward
