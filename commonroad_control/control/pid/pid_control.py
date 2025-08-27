from commonroad_control.control.control import Controller
from commonroad_control.vehicle_dynamics.input_interface import InputInterface
from commonroad_control.vehicle_dynamics.state_interface import StateInterface


class PIDControl(Controller):

    def __init__(
            self,
            kp: float,
            ki: float,
            kd: float,
    ):
        super().__init__()
        self._kp: float = kp
        self._ki: float = ki
        self._kd: float = kd

        self._integrated_error: float = 0.0
        self._previous_error: float = 0.0

    def compute_control_input(
            self,
            measured_state: float,
            desired_state: float,
            controller_time_step: float = 0.01
    ) -> float:
        """
        Computes control output for float input
        :param measured_state:
        :param desired_state:
        :param controller_time_step:
        :return:
        """
        error: float = desired_state - measured_state
        d_error: float = (error - self._previous_error) / controller_time_step
        self._integrated_error += error
        self._previous_error = error

        return self._kp * error + self._ki * self._integrated_error + self._kd * d_error

    def reset(self) -> None:
        self._error_i = 0.0

