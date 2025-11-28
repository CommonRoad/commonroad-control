from commonroad_control.control.control import ControllerInterface


class PIDControl(ControllerInterface):

    def __init__(self, kp: float, ki: float, kd: float, dt: float) -> None:
        """
        PID controller
        :param kp: proportional gain
        :param ki: integral gain
        :param kd: derivative gain
        :param dt: controller step size in seconds
        """
        super().__init__()
        self._kp: float = kp
        self._ki: float = ki
        self._kd: float = kd
        self._dt: float = dt

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
    def dt(self) -> float:
        """
        :return: controller step size in seconds
        """
        return self._dt

    def compute_control_input(
        self,
        measured_state: float,
        desired_state: float,
    ) -> float:
        """
        Computes control output for float input
        :param measured_state: measured single state
        :param desired_state: desired single state
        :return: control input
        """
        error: float = desired_state - measured_state
        d_error: float = (
            error - self._previous_error
        ) / self._dt  # TODO: check if division is reasonable
        self._integrated_error += error * self._dt
        self._previous_error = error

        return self._kp * error + self._ki * self._integrated_error + self._kd * d_error

    def reset(self) -> None:
        """
        Reset running error values
        """
        self._integrated_error = 0.0
        self._previous_error = 0.0
