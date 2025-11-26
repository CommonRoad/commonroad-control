from commonroad_control.control.control import ControllerInterface


class PIDControl(ControllerInterface):

    def __init__(self, kp: float, ki: float, kd: float, dt: float):
        super().__init__()
        self._kp: float = kp
        self._ki: float = ki
        self._kd: float = kd
        self._dt: float = dt

        self._integrated_error: float = 0.0
        self._previous_error: float = 0.0

    def compute_control_input(
        self,
        measured_state: float,
        desired_state: float,
    ) -> float:
        """
        Computes control output for float input
        :param measured_state:
        :param desired_state:
        :return:
        """
        error: float = desired_state - measured_state
        d_error: float = (
            error - self._previous_error
        ) / self._dt  # TODO: check if division is reasonable
        self._integrated_error += error * self._dt
        self._previous_error = error

        return self._kp * error + self._ki * self._integrated_error + self._kd * d_error

    def reset(self) -> None:
        self._integrated_error = 0.0
        self._previous_error = 0.0
