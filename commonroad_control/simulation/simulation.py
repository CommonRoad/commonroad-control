from scipy.integrate import solve_ivp

from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class Simulation:
    def __init__(self, vehicle_model: VehicleModelInterface):

        self._vehicle_model = vehicle_model

    def simulate(self, x0: StateInterface, u: InputInterface, time_horizon: float) -> StateInterface:

        x0_num = x0.convert_to_array()
        u_num = u.convert_to_array()

        x_sim = solve_ivp(lambda t, y : self._vehicle_model._dynamics(y,u_num), [0, time_horizon], y0=x0_num)
        x_end = x_sim[:,-1]

        return x_end
