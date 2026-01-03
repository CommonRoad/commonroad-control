import copy
import logging
from typing import Any, Dict, List, Union

import numpy as np

# reactive planner
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState

# own code base
from commonroad_control.planning_converter.planning_converter_interface import (
    PlanningConverterInterface,
)
from commonroad_control.util.conversion_util import (
    compute_position_of_cog_from_ra_cc,
    compute_position_of_ra_from_cog_cartesian,
    compute_velocity_components_from_steering_angle_in_cog,
    compute_velocity_from_components,
    map_velocity_from_cog_to_ra,
    map_velocity_from_ra_to_cog,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import (
    DBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import (
    DBTrajectory,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory import (
    KBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import (
    KBTrajectory,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams

logger = logging.getLogger(__name__)


class ReactivePlannerConverter(PlanningConverterInterface):
    """
    #TODO add docstring
    """

    def __init__(
        self,
        kb_factory: Union[KBSIDTFactory, Any] = KBSIDTFactory(),
        db_factory: Union[DBSIDTFactory, Any] = DBSIDTFactory(),
        vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams(),
    ) -> None:
        """
        Converter for CommonRoad reactive planner to different vehicle models
        :param kb_factory: kb Factory
        :param db_factory: db Factory
        :param vehicle_params: vehicle params
        """
        super().__init__(
            kb_factory=kb_factory,
            db_factory=db_factory,
            vehicle_params=vehicle_params,
        )

    # --- kb ---
    def trajectory_p2c_kb(
        self,
        planner_traj: Union[List["ReactivePlannerState"], List[InputState]],
        mode: TrajectoryMode,
        t_0: float = 0.0,
        dt: float = 0.1,
    ) -> KBTrajectory:
        """
        Convert reactive planner state or input trajectory to KB trajectory
        :param planner_traj: planner state or input trajectory
        :param mode: state or input mode
        :param t_0: starting time of trajectory
        :param dt: time step size
        :return: KBTrajectory
        """
        kb_dict: Dict[int, Union[KBState, KBInput]] = dict()
        for kb_point in planner_traj:
            kb_dict[kb_point.time_step] = self.sample_p2c_kb(
                planner_state=kb_point, mode=mode
            )
        return self._kb_factory.trajectory_from_points(
            trajectory_dict=kb_dict, mode=mode, t_0=t_0, delta_t=dt
        )

    def sample_p2c_kb(
        self,
        planner_state: Union[ReactivePlannerState, InputState],
        mode: TrajectoryMode,
        *args,
        **kwargs,
    ) -> Union[KBState, KBInput]:
        """
        Convert one state or input of reactive planner to kb
        :param planner_state: planner state or input
        :param mode: state or input
        :return: KBState or KBInput object
        """
        if mode.value == TrajectoryMode.State.value:
            # compute velocity at center of gravity
            v_cog = map_velocity_from_ra_to_cog(
                l_wb=self._vehicle_params.l_wb,
                l_r=self._vehicle_params.l_r,
                velocity_ra=planner_state.velocity,
                steering_angle=planner_state.steering_angle,
            )
            # compute position of the center of gravity
            position_x_cog, position_y_cog = compute_position_of_cog_from_ra_cc(
                position_ra_x=planner_state.position[0],
                position_ra_y=planner_state.position[1],
                heading=planner_state.orientation,
                l_r=self._vehicle_params.l_r,
            )
            retval: KBState = self._kb_factory.state_from_args(
                position_x=position_x_cog,
                position_y=position_y_cog,
                velocity=v_cog,
                heading=planner_state.orientation,
                steering_angle=planner_state.steering_angle,
            )
        elif mode.value == TrajectoryMode.Input.value:
            retval: KBInput = self._kb_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed,
            )
        else:
            logger.error(f"{mode} not implemented")
            raise NotImplementedError(f"{mode} not implemented")

        return retval

    def trajectory_c2p_kb(
        self,
        kb_traj: KBTrajectory,
        mode: TrajectoryMode,
    ) -> Union[List[ReactivePlannerState], List[InputState]]:
        """
        Convert kinematic bicycle state or input trajectory to reactive planner state or input trajectory
        :param kb_traj: KB trajectory
        :param mode: state or input mode
        :return: Reactive planner state or input trajectory
        """
        ordered_points_by_step = dict(sorted(kb_traj.points.items()))
        retval: List[ReactivePlannerState] = list()
        for step, point in ordered_points_by_step.items():
            retval.append(self.sample_c2p_kb(kb_state=point, mode=mode, time_step=step))
        return retval

    def sample_c2p_kb(
        self, kb_state: Union[KBState, KBInput], mode: TrajectoryMode, time_step: int
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Convert kinematic bicycle state or input to reactive planner state or input at time step.
        :param kb_state: KB state or input
        :param mode: state or input mode
        :param time_step: time step
        :return: Reactive planner state or input
        """
        if mode == TrajectoryMode.State:
            # transform velocity to rear axle
            v_ra = map_velocity_from_cog_to_ra(
                l_wb=self._vehicle_params.l_wb,
                l_r=self._vehicle_params.l_r,
                velocity_cog=kb_state.velocity,
                steering_angle=kb_state.steering_angle,
            )

            # compute position of the center of gravity
            position_x_ra, position_y_ra = compute_position_of_ra_from_cog_cartesian(
                position_cog_x=kb_state.position_x,
                position_cog_y=kb_state.position_y,
                heading=kb_state.heading,
                l_r=self._vehicle_params.l_r,
            )

            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([position_x_ra, position_y_ra]),
                velocity=v_ra,
                orientation=kb_state.heading,
                steering_angle=kb_state.steering_angle,
                yaw_rate=0,
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=kb_state.steering_angle_velocity,
                acceleration=kb_state.acceleration,
                time_step=time_step,
            )
        else:
            logger.error(f"mode {mode} not implemented")
            raise NotImplementedError(f"mode {mode} not implemented")
        return retval

    # --- db ---
    def trajectory_p2c_db(
        self,
        planner_traj: Union[List["ReactivePlannerState"], List[InputState]],
        mode: TrajectoryMode,
        t_0: float = 0.0,
        dt: float = 0.1,
    ) -> DBTrajectory:
        """
        Convert reactive planner state or input trajectory to dynamic bicycle trajectory
        :param planner_traj: reactive planner state or input trajectory
        :param mode: state or input mode
        :param t_0: starting time of trajectory
        :param dt: time step size
        :return: Dynamic bicycle state or input trajectory
        """
        db_dict: Dict[int, Union[DBState, DBInput]] = dict()
        for db_point in planner_traj:
            db_dict[db_point.time_step] = self.sample_p2c_db(
                planner_state=db_point, mode=mode
            )
        return self._db_factory.trajectory_from_points(
            trajectory_dict=db_dict, mode=mode, t_0=t_0, delta_t=dt
        )

    def sample_p2c_db(
        self,
        planner_state: Union[ReactivePlannerState, InputState],
        mode: TrajectoryMode,
    ) -> Union[DBState, DBInput]:
        """
        Convert reactive planner state or input to dynamic bicycle state or input
        :param planner_state: reactive planner state or input
        :param mode: state or input mode
        :return: Dynamic bicycle state or input
        """
        if mode == TrajectoryMode.State:
            # compute velocity at center of gravity
            v_cog = map_velocity_from_ra_to_cog(
                l_wb=self._vehicle_params.l_wb,
                l_r=self._vehicle_params.l_r,
                velocity_ra=planner_state.velocity,
                steering_angle=planner_state.steering_angle,
            )
            v_cog_lon, v_cog_lat = (
                compute_velocity_components_from_steering_angle_in_cog(
                    steering_angle=planner_state.steering_angle,
                    velocity_cog=v_cog,
                    l_wb=self.vehicle_params.l_wb,
                    l_r=self.vehicle_params.l_r,
                )
            )
            # compute position of the center of gravity
            position_x_cog, position_y_cog = compute_position_of_cog_from_ra_cc(
                position_ra_x=planner_state.position[0],
                position_ra_y=planner_state.position[1],
                heading=planner_state.orientation,
                l_r=self._vehicle_params.l_r,
            )

            retval: DBState = self._db_factory.state_from_args(
                position_x=position_x_cog,
                position_y=position_y_cog,
                velocity_long=v_cog_lon,
                velocity_lat=v_cog_lat,
                yaw_rate=planner_state.yaw_rate,
                steering_angle=planner_state.steering_angle,
                heading=planner_state.orientation,
            )
        elif mode == TrajectoryMode.Input:
            retval: DBInput = self._db_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed,
            )
        else:
            logger.error(f"mode {mode} not implemented")
            raise NotImplementedError(f"mode {mode} not implemented")

        return retval

    def trajectory_c2p_db(self, db_traj: DBTrajectory, mode: TrajectoryMode) -> Any:
        """
        NOT IMPLEMENTED!
        :param db_traj:
        :param mode:
        :return:
        """
        logger.error("Currently not implemented")
        raise NotImplementedError("Currently not implemented")

    def sample_c2p_db(
        self,
        db_state: Union[DBState, DBInput],
        mode: TrajectoryMode,
        time_step: int,
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Convert dynamic bycicle state or input to reactive planner state or input at time step.
        :param db_state: Dynamic bicycle state or input
        :param mode: state or input mode
        :param time_step: time step
        :return: reactive planner state or input
        """
        if mode == TrajectoryMode.State:
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([db_state.position_x, db_state.position_y]),
                velocity=compute_velocity_from_components(
                    db_state.velocity_long, db_state.velocity_lat
                ),
                orientation=db_state.heading,
                steering_angle=db_state.steering_angle,
                yaw_rate=db_state.yaw_rate,
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=db_state.steering_angle_velocity,
                acceleration=db_state.acceleration,
                time_step=time_step,
            )
        return retval

    @staticmethod
    def time_shift_state_input_list(
        si_list: Union[List[ReactivePlannerState], List[InputState]], t_0: int
    ) -> Union[List[ReactivePlannerState], List[InputState]]:
        """
        Time-shift list of reactive planner states or inputs to initial state.
        Updates each entries .time_step attribute
        :param si_list: state or input list
        :param t_0: new initial time step
        :return: new instance of reactive planenr state or input list
        """
        time_shifted_si_list = copy.copy(si_list)
        for idx in range(len(si_list)):
            time_shifted_si_list[idx].time_step = idx + t_0

        return time_shifted_si_list
