from typing import Union, Any, Literal, List, Dict

from math import  tan, sqrt, cos, sin
import numpy as np
# reactive planner
from commonroad_idm_planner.idm_trajectory import IDMTrajectory
from commonroad_idm_planner.idm_state import IDMState
from commonroad_idm_planner.idm_input import IDMInput
from commonroad_idm_planner.util.geometry import wrap_to_pi

# own code base
from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
from commonroad_control.util.conversion_util import (
    compute_velocity_components_from_steering_angle_in_cog,
    compute_total_velocity_from_components,
    map_velocity_from_ra_to_cog,
    compute_position_of_cog_from_ra_cc,
    map_velocity_from_cog_to_ra,
    compute_position_of_ra_from_cog_cartesian
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



class IDMPlannerConverter(PlanningConverterInterface):
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

    def __init__(
            self,
            config: int = 0,
            kb_factory: Union[KBSITFactory, Any] = KBSITFactory(),
            db_factory: Union[DBSITFactory, Any] = DBSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        """
        Converter for CommonRoad idm planner
        :param config: dummy
        :param kb_factory: kb Factory
        :param db_factory: db Factory
        :param vehicle_params: vehicle params
        """
        super().__init__(
            config=config,
            kb_factory=kb_factory,
            db_factory=db_factory,
            vehicle_params=vehicle_params
        )

    # --- kb ---
    def trajectory_p2c_kb(
            self,
            planner_traj: Union[IDMTrajectory, List[IDMInput]],
            mode: TrajectoryMode,
            t_0: float = 0.0,
            dt: float = 0.1
    ) -> KBTrajectory:
        """
        Convert IDM Planner Trajectory To kb-Trajectory
        :param planner_traj: state or input trajectory
        :param mode: state or input mode
        :param t_0: starting time of trajectory
        :param dt: time step size
        :return: KBTrajectory
        """
        kb_dict: Dict[int, Union[KBState, KBInput]] = dict()
        p_traj = planner_traj if mode == TrajectoryMode.Input else planner_traj.state_list
        for idm_point in p_traj:
            kb_dict[idm_point.time_step] = self.sample_p2c_kb(
                planner_state=idm_point,
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
            planner_state: Union[IDMState, IDMInput],
            mode: TrajectoryMode,
    ) -> Union[KBState, KBInput]:
        """
        Convert one state or input of IDM planner to kb
        :param planner_state: planner state or input
        :param mode: state or input
        :return: KBState or KBInput object
        """
        if mode.value == TrajectoryMode.State.value:
            # idm planner already has position on COG
            retval: KBState = self._kb_factory.state_from_args(
                position_x=planner_state.position[0],
                position_y=planner_state.position[1],
                velocity=planner_state.velocity,
                heading=planner_state.orientation,
                steering_angle=planner_state.steering_angle
            )
        elif mode.value == TrajectoryMode.Input.value:
            retval: KBInput = self._kb_factory.input_from_args(
                acceleration=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_velocity,
            )
        else:
            raise NotImplementedError(f"{mode} not implemented")

        return retval


    def trajectory_c2p_kb(
            self,
            kb_traj: KBTrajectory,
            mode: TrajectoryMode,
    ) -> Union[IDMTrajectory, List[IDMInput]]:
        raise NotImplementedError()

    def sample_c2p_kb(
            self,
            kb_state: Union[KBState, KBInput],
            mode: TrajectoryMode,
            time_step: int
    ) -> Union[IDMState, IDMInput]:
        raise NotImplementedError()

    # --- db ---
    def trajectory_p2c_db(
            self,
            planner_traj: Union[IDMTrajectory, List[IDMState]],
            mode: TrajectoryMode,
            t_0: float = 0.0,
            dt: float = 0.1
    ) -> DBTrajectory:
        """
        Build dynamic-single-track trajectory from IDM planner
        :param planner_traj:
        :param mode:
        :param t_0:
        :param dt:
        :return: DBTrajectory
        """
        db_dict: Dict[int, Union[DBState, DBInput]] = dict()

        # add yaw rate attribute to idm state if necessary
        if mode == TrajectoryMode.State and not hasattr(planner_traj.state_list[0], "yaw_rate"):
            setattr(planner_traj.state_list[0], "yaw_rate", 0.0)
            for idx in range(len(planner_traj.state_list) - 2):
                yaw_rate: float = (planner_traj[idx + 1].orientation - planner_traj[idx].orientation) / dt
                setattr(planner_traj[idx + 1], "yaw_rate", yaw_rate)

        p_traj = planner_traj if mode == TrajectoryMode.Input else planner_traj.state_list

        for db_point in p_traj:
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
            planner_state: Union[IDMState, IDMInput],
            mode: TrajectoryMode
    ) -> Union[DBState, DBInput]:
        """
        Create dynamic-single-track state or input from reactive planner
        :param planner_state:
        :param mode:
        :return: DBState or DBInput
        """
        if mode == TrajectoryMode.State:
            # IDM already is in COG
            # compute velocity at center of gravity
            v_cog = planner_state.velocity
            v_cog_lon, v_cog_lat = compute_velocity_components_from_steering_angle_in_cog(
                steering_angle=planner_state.steering_angle,
                velocity_cog=v_cog,
                l_wb=self.vehicle_params.l_wb,
                l_r=self.vehicle_params.l_r
            )

            retval: DBState = self._db_factory.state_from_args(
                position_x=planner_state.position[0],
                position_y=planner_state.position[1],
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
        else:
            raise NotImplementedError(f"mode {mode} not implemented")

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
    ) -> Union[IDMState, IDMInput]:
        """
        Convert control output to Reactive planner
        :param db_state:
        :param mode: choose between state and input
        :param time_step:
        :return: ReactivePlannerState or InputState
        """
        raise NotImplementedError()
