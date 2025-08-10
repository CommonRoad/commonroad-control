from typing import Union, Any, Literal, List, Dict

import numpy as np
# reactive planner
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState

# own code base
from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
from commonroad_control.util.conversion_util import (
    compute_velocity_components_from_steering_angle_in_cog,
    compute_total_velocity_from_components
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class ReactivePlannerConverter(PlanningConverterInterface):
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

    def __init__(
            self,
            config: int = 0,
            kst_factory: Union[KSTSITFactory, Any] = KSTSITFactory(),
            dst_factory: Union[DBSITFactory, Any] = DBSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        """
        Converter for CommonRoad reactive planner
        :param config: dummy
        :param kst_factory: KST Factory
        :param dst_factory: DST Factory
        :param vehicle_params: vehicle params
        """
        super().__init__(
            config=config,
            kst_factory=kst_factory,
            dst_factory=dst_factory,
            vehicle_params=vehicle_params
        )

    # --- KST ---
    def trajectory_p2c_kst(
            self,
            planner_traj: Union[List['ReactivePlannerState'], List[InputState]],
            mode: TrajectoryMode,
            t_0: float = 0.0,
            dt: float = 0.1
    ) -> KSTTrajectory:
        """
        Convert Reactive Planner Trajectory To KST-Trajectory
        :param planner_traj: state or input trajectory
        :param mode: state or input mode
        :param t_0: starting time of trajectory
        :param dt: time step size
        :return: KSTTrajectory
        """
        kst_dict: Dict[int, Union[KSTState, KSTInput]] = dict()
        for kst_point in planner_traj:
            kst_dict[kst_point.time_step] = self.sample_p2c_kst(
                planner_state=kst_point,
                mode=mode
            )
        return self._kst_factory.trajectory_from_state_or_input(
            trajectory_dict=kst_dict,
            mode=mode,
            t_0=t_0,
            delta_t=dt
        )

    def sample_p2c_kst(
            self,
            planner_state: Union[ReactivePlannerState, InputState],
            mode: TrajectoryMode
    ) -> Union[KSTState, KSTInput]:
        """
        Convert one state or input of reactive planner to kst
        :param planner_state: planner state or input
        :param mode: state or input
        :return: KSTState or KSTInput object
        """
        # TODO: Double-check -> ReactivePlanner has position on COG but velocity is rear axle
        if mode == TrajectoryMode.State:
            retval: KSTState = self._kst_factory.state_from_args(
                position_x=planner_state.position[0],
                position_y=planner_state.position[1],
                velocity=planner_state.velocity,
                heading=planner_state.orientation,
                steering_angle=planner_state.steering_angle
            )
        elif mode == TrajectoryMode.Input:
            retval: KSTInput = self._kst_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed,
            )

        return retval



    def trajectory_c2p_kst(
            self,
            kst_traj: KSTTrajectory,
            mode: TrajectoryMode,
    ) -> Union[List[ReactivePlannerState], List[InputState]]:
        ordered_points_by_step = dict(sorted(kst_traj.points.items()))
        retval: List[ReactivePlannerState] = list()
        for step, point in ordered_points_by_step.items():
            retval.append(
                self.sample_c2p_kst(
                    kst_state=point,
                    mode=mode,
                    time_step=step)
            )
        return retval

    def sample_c2p_kst(
            self,
            kst_state: Union[KSTState, KSTInput],
            mode: TrajectoryMode,
            time_step: int
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Get Reactive planner state or input from kst state or input
        :param kst_state:
        :param mode:
        :param time_step:
        :return:
        """
        if mode == TrajectoryMode.State:
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([kst_state.position_x, kst_state.position_y]),
                velocity=kst_state.velocity,
                orientation=kst_state.heading,
                steering_angle=kst_state.steering_angle,
                yaw_rate=0
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=kst_state.steering_angle_velocity,
                acceleration=kst_state.acceleration,
                time_step=time_step
            )
        return retval

    # --- DST ---
    def trajectory_p2c_dst(
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
        dst_dict: Dict[int, Union[DBState, DBInput]] = dict()
        for dst_point in planner_traj:
            dst_dict[dst_point.time_step] = self.sample_p2c_dst(
                planner_state=dst_point,
                mode=mode
            )
        return self._dst_factory.trajectory_from_state_or_input(
            trajectory_dict=dst_dict,
            mode=mode,
            t_0=t_0,
            delta_t=dt
        )



    def sample_p2c_dst(
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
            v_lon, v_lat = compute_velocity_components_from_steering_angle_in_cog(
                steering_angle=planner_state.steering_angle,
                velocity=planner_state.velocity,
                wheelbase=self.vehicle_params.l_wb,
                length_rear=self.vehicle_params.l_r
            )

            retval: DBState = self._dst_factory.state_from_args(
                position_x=planner_state.position[0],
                position_y=planner_state.position[1],
                velocity_long=v_lon,
                velocity_lat=v_lat,
                yaw_rate=planner_state.yaw_rate,
                steering_angle=planner_state.steering_angle,
                heading=planner_state.orientation
            )
        elif TrajectoryMode.Input:
            retval: DBInput = self._dst_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed
            )

        return retval


    def trajectory_c2p_dst(
            self,
            dst_traj: DBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        raise NotImplementedError("Currently not implemented")


    def sample_c2p_dst(
            self,
            dst_state: Union[DBState, DBInput],
            mode: TrajectoryMode,
            time_step: int,
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Convert controller output to Reactive planner
        :param dst_state:
        :param mode: choose between state and input
        :param time_step:
        :return: ReactivePlannerState or InputState
        """
        if mode == TrajectoryMode.State:
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([dst_state.position_x, dst_state.position_y]),
                velocity=compute_total_velocity_from_components(
                    dst_state.velocity_long, dst_state.velocity_lat
                ),
                orientation=dst_state.heading,
                steering_angle=dst_state.steering_angle,
                yaw_rate=dst_state.yaw_rate
            )
        elif mode == TrajectoryMode.Input:
            retval: InputState = InputState(
                steering_angle_speed=dst_state.steering_angle_velocity,
                acceleration=dst_state.acceleration,
                time_step=time_step
            )
        return retval
