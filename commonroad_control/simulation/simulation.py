from typing import Union, Any

from scipy.integrate import solve_ivp

from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.vehicle_model_interface import VehicleModelInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters


class Simulation:
    def __init__(
            self,
            vehicle_model: VehicleModelInterface,
            state_input_factory: Union[DISITFactory, KSTSITFactory, DBSITFactory]
    ):

        self._vehicle_model: VehicleModelInterface = vehicle_model
        self._state_input_factory: Union[DISITFactory, KSTSITFactory, DBSITFactory] = state_input_factory

    def simulate(self, x0: StateInterface, u: InputInterface, time_horizon: float) -> Union[StateInterface, Any]:

        x0_num = x0.convert_to_array()
        u_num = u.convert_to_array()

        # TODO fix to public var
        res_sim = solve_ivp(lambda t, y: self._vehicle_model._dynamics_ct(y, u_num), [0, time_horizon], y0=x0_num,
                          method='RK45')
        x_end = res_sim.y[:, -1]

        # TODO: convert to state interface
        x_forward = self._state_input_factory.state_from_numpy_array(x_np=x_end)


        return x_forward
