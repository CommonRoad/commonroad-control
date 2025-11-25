from typing import Union, Tuple, List

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface, TrajectoryMode
from commonroad_control.vehicle_dynamics.sidt_factory_interface import StateInputDisturbanceTrajectoryFactoryInterface


class ReferenceTrajectoryFactory:
    def __init__(self,
                 delta_t_controller: float,
                 sit_factory: StateInputDisturbanceTrajectoryFactoryInterface,
                 horizon: int = 0,
                 t_look_ahead: float = 0.0):

        self._horizon: int = horizon
        self._dt_controller: float = delta_t_controller
        self._t_look_ahead: float = t_look_ahead
        self._sit_factory: StateInputDisturbanceTrajectoryFactoryInterface = sit_factory

        self._x_ref: Union[TrajectoryInterface, None] = None
        self._u_ref: Union[TrajectoryInterface, None] = None
        if horizon > 0:
            self._x_ref_steps: List[int] = [kk for kk in range(self._horizon + 1)]
            self._u_ref_steps: List[int] = [kk for kk in range(self._horizon)]
        else:
            self._x_ref_steps = [0]
            self._u_ref_steps = [0]


    @property
    def state_trajectory(self) -> Union[TrajectoryInterface, None]:
        """
        :return: state trajectory
        """
        return self._x_ref


    def set_reference_trajectory(self,
                                 state_ref: TrajectoryInterface,
                                 input_ref: TrajectoryInterface,
                                 t_0: float) -> None:
        """
        Function for updating the reference trajectory (e.g. given a new planned trajectory).
        :param state_ref: state reference trajectory
        :param input_ref: input reference trajectory
        :param t_0: initial time step of the reference trajectories
        """

        # consistency checks
        if state_ref.mode is not TrajectoryMode.State:
            raise TypeError(f"Invalid mode of state reference trajectory: expected {TrajectoryMode.State}")
        if input_ref.mode is not TrajectoryMode.Input:
            raise TypeError(f"Invalid mode of input reference trajectory: expected {TrajectoryMode.Input}")
        if abs(state_ref.t_0-t_0) > 1e-12or abs(input_ref.t_0-t_0) > 1e-12:
            raise ValueError(f"Inconsistent initial time for state and/or reference input trajectory")
        if t_0 + self._t_look_ahead + max(self._x_ref_steps)*self._dt_controller > state_ref.t_final or \
            t_0 + self._t_look_ahead + max(self._u_ref_steps)*self._dt_controller > input_ref.t_final:
            raise ValueError(f"Prediction/look-ahead horizon exceeds the final time of the given reference trajectories")

        self._x_ref = state_ref
        self._u_ref = input_ref



    def get_reference_trajectory_at_time(self,
                                         t: float) \
        -> Tuple[TrajectoryInterface, TrajectoryInterface]:
        """
        Returns a snippet from the reference trajectory (single or multiple points, depending on the controller, which
        is passed to the controller for computing the control inputs.
        :param t: current time
        :return: state and input reference trajectory
        """

        # ... extract state reference trajectory
        t_0 = t + self._t_look_ahead
        tmp_x_ref_points = [self._x_ref.get_point_at_time(t_0 + kk*self._dt_controller, self._sit_factory)
                            for kk in self._x_ref_steps]
        x_ref = self._sit_factory.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(
                self._x_ref_steps, tmp_x_ref_points)),
            mode=TrajectoryMode.State,
            t_0=t_0,
            delta_t=self._dt_controller
        )
        # ... extract input reference trajectory
        tmp_u_ref_points = [self._u_ref.get_point_at_time(t_0 + kk*self._dt_controller, self._sit_factory)
                            for kk in self._u_ref_steps]
        u_ref = self._sit_factory.trajectory_from_state_or_input(
            trajectory_dict=dict(zip(
                self._u_ref_steps, tmp_u_ref_points)),
            mode=TrajectoryMode.Input,
            t_0=t_0,
            delta_t=self._dt_controller
        )

        return x_ref, u_ref
