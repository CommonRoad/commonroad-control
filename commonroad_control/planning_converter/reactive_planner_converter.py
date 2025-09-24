from typing import Union, Any, Literal, List, Dict

from math import  tan, sqrt, cos, sin
import numpy as np
# reactive planner
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState

# own code base
from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
from commonroad_control.util.conversion_util import (
    compute_velocity_components_from_steering_angle_in_cog,
    compute_total_velocity_from_components,
    map_velocity_from_ra_to_cog,
    compute_position_of_cog_from_ra_cc
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


# TODO: read position of rear-axle from reactive planner

class ReactivePlannerConverter(PlanningConverterInterface):
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

    def __init__(
            self,
            config: int = 0,
            kb_factory: Union[KBSITFactory, Any] = KBSITFactory(),
            dst_factory: Union[DBSITFactory, Any] = DBSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        """
        Converter for CommonRoad reactive planner
        :param config: dummy
        :param kb_factory: kb Factory
        :param dst_factory: DST Factory
        :param vehicle_params: vehicle params
        """
        super().__init__(
            config=config,
            kb_factory=kb_factory,
            dst_factory=dst_factory,
            vehicle_params=vehicle_params
        )

    # --- kb ---
    def trajectory_p2c_kb(
            self,
            planner_traj: Union[List['ReactivePlannerState'], List[InputState]],
            mode: TrajectoryMode,
            t_0: float = 0.0,
            dt: float = 0.1
    ) -> KBTrajectory:
        """
        Convert Reactive Planner Trajectory To kb-Trajectory
        :param planner_traj: state or input trajectory
        :param mode: state or input mode
        :param t_0: starting time of trajectory
        :param dt: time step size
        :return: KBTrajectory
        """
        kb_dict: Dict[int, Union[KBState, KBInput]] = dict()
        for kb_point in planner_traj:
            kb_dict[kb_point.time_step] = self.sample_p2c_kb(
                planner_state=kb_point,
                mode=mode
            )
        return self._kb_factory.trajectory_from_state_or_input(
            trajectory_dict=kb_dict,
            mode=mode,
            t_0=t_0,
            delta_t=dt
        )

    def sample_p2c_kb(
            self,
            planner_state: Union[ReactivePlannerState, InputState],
            mode: TrajectoryMode
    ) -> Union[KBState, KBInput]:
        """
        Convert one state or input of reactive planner to kb
        :param planner_state: planner state or input
        :param mode: state or input
        :return: KBState or KBInput object
        """
        if mode == TrajectoryMode.State:
            # compute velocity at center of gravity
            v_cog = map_velocity_from_ra_to_cog(
                l_wb=self._vehicle_params.l_wb,
                l_r=self._vehicle_params.l_r,
                velocity_ra=planner_state.velocity,
                steering_angle=planner_state.steering_angle
            )
            # compute position of the center of gravity
            position_x_cog, position_y_cog = compute_position_of_cog_from_ra_cc(
                position_ra_x=planner_state.position[0],
                position_ra_y=planner_state.position[1],
                heading=planner_state.orientation,
                l_r=self._vehicle_params.l_r
            )
            retval: KBState = self._kb_factory.state_from_args(
                position_x=position_x_cog,
                position_y=position_y_cog,
                velocity=v_cog,
                heading=planner_state.orientation,
                steering_angle=planner_state.steering_angle
            )
        elif mode == TrajectoryMode.Input:
            retval: KBInput = self._kb_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed,
            )

        return retval



    def trajectory_c2p_kb(
            self,
            kb_traj: KBTrajectory,
            mode: TrajectoryMode,
    ) -> Union[List[ReactivePlannerState], List[InputState]]:
        ordered_points_by_step = dict(sorted(kb_traj.points.items()))
        retval: List[ReactivePlannerState] = list()
        for step, point in ordered_points_by_step.items():
            retval.append(
                self.sample_c2p_kb(
                    kb_state=point,
                    mode=mode,
                    time_step=step)
            )
        return retval

    def sample_c2p_kb(
            self,
            kb_state: Union[KBState, KBInput],
            mode: TrajectoryMode,
            time_step: int
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Get Reactive planner state or input from kb state or input
        :param kb_state:
        :param mode:
        :param time_step:
        :return:
        """
        #TODO check conversion
        if mode == TrajectoryMode.State:
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([kb_state.position_x, kb_state.position_y]),
                velocity=kb_state.velocity,
                orientation=kb_state.heading,
                steering_angle=kb_state.steering_angle,
                yaw_rate=0
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=kb_state.steering_angle_velocity,
                acceleration=kb_state.acceleration,
                time_step=time_step
            )
        return retval

    # --- db ---
    def trajectory_p2c_db(
            self,
            planner_traj: Union[List['ReactivePlannerState'], List[InputState]],
            mode: TrajectoryMode,
            t_0: float = 0.0,
            dt: float = 0.1
    ) -> DBTrajectory:
        """
        Build dynamic-single-track trajectory from reactive planner
        :param planner_traj:
        :param mode:
        :param t_0:
        :param dt:
        :return: DBTrajectory
        """
        db_dict: Dict[int, Union[DBState, DBInput]] = dict()
        for db_point in planner_traj:
            db_dict[db_point.time_step] = self.sample_p2c_db(
                planner_state=db_point,
                mode=mode
            )
        return self._db_factory.trajectory_from_state_or_input(
            trajectory_dict=db_dict,
            mode=mode,
            t_0=t_0,
            delta_t=dt
        )



    def sample_p2c_db(
            self,
            planner_state: Union[ReactivePlannerState, InputState],
            mode: TrajectoryMode
    ) -> Union[DBState, DBInput]:
        """
        Create dynamic-single-track state or input from reactive planner
        :param planner_state:
        :param mode:
        :return: DBState or DBInput
        """
        if mode == TrajectoryMode.State:
            # compute velocity at center of gravity
            v_cog = map_velocity_from_ra_to_cog(
                l_wb=self._vehicle_params.l_wb,
                l_r=self._vehicle_params.l_r,
                velocity_ra=planner_state.velocity,
                steering_angle=planner_state.steering_angle
            )
            v_cog_lon, v_cog_lat = compute_velocity_components_from_steering_angle_in_cog(
                steering_angle=planner_state.steering_angle,
                velocity_cog=v_cog,
                l_wb=self.vehicle_params.l_wb,
                l_r=self.vehicle_params.l_r
            )
            # compute position of the center of gravity
            position_x_cog, position_y_cog = compute_position_of_cog_from_ra_cc(
                position_ra_x=planner_state.position[0],
                position_ra_y=planner_state.position[1],
                heading=planner_state.orientation,
                l_r=self._vehicle_params.l_r
            )

            retval: DBState = self._db_factory.state_from_args(
                position_x=position_x_cog,
                position_y=position_y_cog,
                velocity_long=v_cog_lon,
                velocity_lat=v_cog_lat,
                yaw_rate=planner_state.yaw_rate,
                steering_angle=planner_state.steering_angle,
                heading=planner_state.orientation
            )
        elif mode == TrajectoryMode.Input:
            retval: DBInput = self._db_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed
            )

        return retval


    def trajectory_c2p_db(
            self,
            db_traj: DBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        raise NotImplementedError("Currently not implemented")


    def sample_c2p_db(
            self,
            db_state: Union[DBState, DBInput],
            mode: TrajectoryMode,
            time_step: int,
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Convert control output to Reactive planner
        :param db_state:
        :param mode: choose between state and input
        :param time_step:
        :return: ReactivePlannerState or InputState
        """
        #TODO check conversion
        if mode == TrajectoryMode.State:
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([db_state.position_x, db_state.position_y]),
                velocity=compute_total_velocity_from_components(
                    db_state.velocity_long, db_state.velocity_lat
                ),
                orientation=db_state.heading,
                steering_angle=db_state.steering_angle,
                yaw_rate=db_state.yaw_rate
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=db_state.steering_angle_velocity,
                acceleration=db_state.acceleration,
                time_step=time_step
            )
        return retval
