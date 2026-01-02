from commonroad_control.control.control import ControllerInterface


class PIDControl(ControllerInterface):

    def __init__(self, kp: float, ki: float, kd: float, delta_t: float) -> None:
        """
        PID controller
        :param kp: proportional gain
        :param ki: integral gain
        :param kd: derivative gain
        :param delta_t: controller sampling time in seconds
        """
        super().__init__()
        self._kp: float = kp
        self._ki: float = ki
        self._kd: float = kd
        self._delta_t: float = delta_t

        self._integrated_error: float = 0.0
        self._previous_error: float = 0.0

    @property
    def kp(self) -> float:
        """
        :return: proportional gain
        """
        return self._kp

    @property
    def ki(self) -> float:
        """
        :return: integral gain
        """
        return self._ki

    @property
    def kd(self) -> float:
        """
        :return: derivative gain
        """
        return self._kd

    @property
    def delta_t(self) -> float:
        """
        :return: controller sampling time in seconds
        """
        return self._delta_t

    def compute_control_input(
        self,
        measured_output: float,
        reference_output: float,
    ) -> float:
        """
        Computes control output for float input
        :param measured_output: measured single output
        :param reference_output: reference single output
        :return: control input
        """
        error: float = reference_output - measured_output
        d_error: float = (
            error - self._previous_error
        ) / self._delta_t
        self._integrated_error += error * self._delta_t
        self._previous_error = error

        return self._kp * error + self._ki * self._integrated_error + self._kd * d_error

    def reset(self) -> None:
        """
        Reset running error values
        """
        self._integrated_error = 0.0
        self._previous_error = 0.0
