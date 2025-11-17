from typing import Any, Tuple,Callable

from commonroad_control.control.pid.pid_control import PIDControl
from commonroad_control.control.control import ControllerInterface


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
            kp_steer_heading: float,
            ki_steer_heading: float,
            kd_steer_heading: float,
            kp_steer_offset: float,
            ki_steer_offset: float,
            kd_steer_offset: float,
            dt: float
    ) -> None:
        """
        PID-based controller that combines a PID for longitudinal acceleration based on longitudinal velocity error
        and two PID-based parts for steering control, one controlling the heading error and one the lateral offset
        :param kp_long:
        :param ki_long:
        :param kd_long:
        :param kp_steer_heading:
        :param ki_steer_heading:
        :param kd_steer_heading:
        :param kp_steer_offset:
        :param ki_steer_offset:
        :param kd_steer_offset:
        :param dt:
        :param look_ahead_simulation:
        :param look_ahead_s:
        """
        super().__init__()
        self._v_long_pid: PIDControl = PIDControl(
            kp=kp_long,
            ki=ki_long,
            kd=kd_long,
            dt=dt
        )
        self._steer_pid_heading: PIDControl = PIDControl(
            kp=kp_steer_heading,
            ki=ki_steer_heading,
            kd=kd_steer_heading,
            dt=dt
        )
        self._steer_pid_offset: PIDControl = PIDControl(
            kp=kp_steer_offset,
            ki=ki_steer_offset,
            kd=kd_steer_offset,
            dt=dt
        )

    def compute_control_input(
        self,
        measured_v_long: float,
        desired_v_long: float,
        measured_heading: float,
        desired_heading: float,
        measured_lat_offset: float,
        desired_lat_offset: float = 0
    ) -> Tuple[float, float]:
        """
        Computes input from controller given desired states and measured states
        :param measured_v_long: measured longitudinal velocity
        :param desired_v_long: desired longitudinal velocity
        :param measured_heading: measured heading
        :param desired_heading: desired heading
        :param measured_lat_offset: measured lateral offset
        :param desired_lat_offset: desired lateral offset, default 0
        :return: Tuple[u_acc, u_steer_vel]
        """
        u_acc: float = self._v_long_pid.compute_control_input(
            measured_state=measured_v_long,
            desired_state=desired_v_long
        )
        u_steer_head: float = self._steer_pid_heading.compute_control_input(
            measured_state=measured_heading,
            desired_state=desired_heading
        )
        u_steer_lat_offset: float = self._steer_pid_offset.compute_control_input(
            measured_state=measured_lat_offset,
            desired_state=desired_lat_offset
        )

        return u_acc, u_steer_head + u_steer_lat_offset

