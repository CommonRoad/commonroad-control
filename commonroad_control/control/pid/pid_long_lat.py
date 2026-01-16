from typing import Tuple

from commonroad_control.control.control import ControllerInterface
from commonroad_control.control.pid.pid_control import PIDControl


class PIDLongLat(ControllerInterface):
    """
    PID-based controller that combines two PID controllers for decoupled longitudinal and lateral control. The longitudinal controller
    tracks a given velocity profile (by adapting the long. acceleration) and the lateral controller reduces the
    lateral offset from the reference trajectory (by adapting the steering angle velocity).
    """

    def __init__(
        self,
        kp_long: float,
        ki_long: float,
        kd_long: float,
        kp_lat: float,
        ki_lat: float,
        kd_lat: float,
        delta_t: float,
    ) -> None:
        """
        Initialize controller.
        :param kp_long: proportional gain longitudinal velocity
        :param ki_long: integral gain longitudinal velocity
        :param kd_long: derivative gain longitudinal velocity
        :param kp_lat: proportional gain lateral offset
        :param ki_lat: integral gain lateral offset
        :param kd_lat: derivative gain lateral offset
        :param delta_t: controller sampling time in seconds
        """
        super().__init__()
        self._v_long_pid: PIDControl = PIDControl(
            kp=kp_long, ki=ki_long, kd=kd_long, delta_t=delta_t
        )

        self._steer_pid_offset: PIDControl = PIDControl(
            kp=kp_lat, ki=ki_lat, kd=kd_lat, delta_t=delta_t
        )
        self._delta_t: float = delta_t

    @property
    def longitudinal_pid(self) -> PIDControl:
        """
        :return: longitudinal PID controller
        """
        return self._v_long_pid

    @property
    def lateral_pid(self) -> PIDControl:
        """
        :return: lateral PID controller
        """
        return self._steer_pid_offset

    @property
    def delta_t(self) -> float:
        """
        :return: controller sampling time in seconds used for both PID controllers
        """
        return self._delta_t

    def compute_control_input(
        self,
        measured_v_long: float,
        reference_v_long: float,
        measured_lat_offset: float,
        reference_lat_offset: float = 0,
    ) -> Tuple[float, float]:
        """
        Computes input from controller given the measured and the reference outputs
        :param measured_v_long: measured longitudinal velocity
        :param reference_v_long: reference longitudinal velocity
        :param measured_lat_offset: measured lateral offset
        :param reference_lat_offset: reference lateral offset, default 0
        :return: control input for acceleration, control input for steering angle velocity
        """
        u_acc: float = self._v_long_pid.compute_control_input(
            measured_output=measured_v_long, reference_output=reference_v_long
        )
        u_steer_lat_offset: float = self._steer_pid_offset.compute_control_input(
            measured_output=measured_lat_offset, reference_output=reference_lat_offset
        )

        return u_acc, u_steer_lat_offset

    def reset(self) -> None:
        """
        Resets internal running states
        """
        self._v_long_pid.reset()
        self._steer_pid_offset.reset()
