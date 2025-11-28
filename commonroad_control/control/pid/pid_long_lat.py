from typing import Tuple

from commonroad_control.control.control import ControllerInterface
from commonroad_control.control.pid.pid_control import PIDControl


class PIDLongLat(ControllerInterface):
    """
    PID-based controller that combines a PID for longitudinal acceleration based on longitudinal velocity error
    and two PID-based parts for lateral control, one controlling the heading error and one the lateral offset
    """

    def __init__(
        self,
        kp_long: float,
        ki_long: float,
        kd_long: float,
        kp_steer_offset: float,
        ki_steer_offset: float,
        kd_steer_offset: float,
        dt: float,
    ) -> None:
        """
        PID-based controller that combines a PID for longitudinal acceleration based on longitudinal velocity error
        and two PID-based parts for steering control, one controlling the heading error and one the lateral offset
        :param kp_long: proportional gain longitudinal velocity
        :param ki_long: integral gain longitudinal velocity
        :param kd_long: derivative gain longitudinal velocity
        :param kp_steer_offset: proportional gain lateral offset
        :param ki_steer_offset: integral gain lateral offset
        :param kd_steer_offset: derivative gain lateral offset
        :param dt: controller step size in seconds
        """
        super().__init__()
        self._v_long_pid: PIDControl = PIDControl(
            kp=kp_long, ki=ki_long, kd=kd_long, dt=dt
        )

        self._steer_pid_offset: PIDControl = PIDControl(
            kp=kp_steer_offset, ki=ki_steer_offset, kd=kd_steer_offset, dt=dt
        )
        self._dt: float = dt

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
    def dt(self) -> float:
        """
        :return: controller step size in seconds used for both PID controllers
        """
        return self._dt

    def compute_control_input(
        self,
        measured_v_long: float,
        desired_v_long: float,
        measured_lat_offset: float,
        desired_lat_offset: float = 0,
    ) -> Tuple[float, float]:
        """
        Computes input from controller given desired states and measured states
        :param measured_v_long: measured longitudinal velocity
        :param desired_v_long: desired longitudinal velocity
        :param measured_lat_offset: measured lateral offset
        :param desired_lat_offset: desired lateral offset, default 0
        :return: controller input for acceleration, controller input for steering angle velocity
        """
        u_acc: float = self._v_long_pid.compute_control_input(
            measured_state=measured_v_long, desired_state=desired_v_long
        )
        u_steer_lat_offset: float = self._steer_pid_offset.compute_control_input(
            measured_state=measured_lat_offset, desired_state=desired_lat_offset
        )

        return u_acc, u_steer_lat_offset

    def reset(self) -> None:
        """
        Resets internal running states
        """
        self._v_long_pid.reset()
        self._steer_pid_offset.reset()
