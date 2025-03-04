from typing import Union, Any, Literal, List, Dict

import numpy as np
# reactive planner
from commonroad.scenario.state import InputState
from commonroad_rp.state import ReactivePlannerState


# own code base
from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
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
            mode: Literal['state', 'input'],
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
            kst_dict=kst_dict,
            mode=mode,
            t_0=t_0,
            delta_t=dt
        )

    def sample_p2c_kst(
            self,
            planner_state: Union[ReactivePlannerState, InputState],
            mode: Literal['state', 'input']
    ) -> Union[KSTState, KSTInput]:
        """
        Convert one state or input of reactive planner to kst
        :param planner_state: planner state or input
        :param mode: state or input
        :return: KSTState or KSTInput object
        """
        if mode == 'state':
            retval: KSTState = self._kst_factory.state_from_args(
                position_x=planner_state.position[0],
                position_y=planner_state.position[1],
                velocity=planner_state.velocity,
                acceleration=planner_state.acceleration,
                heading=planner_state.orientation,
                steering_angle=planner_state.steering_angle
            )
        else:
            retval: KSTInput = self._kst_factory.input_from_args(
                jerk=planner_state.acceleration,
                steering_angle_velocity=planner_state.steering_angle_speed,
            )

        return retval



    def trajectory_c2p_kst(
            self,
            kst_traj: KSTTrajectory,
            mode: Literal['state', 'input'],
    ) -> Union[List[ReactivePlannerState], List[InputState]]:
        ordered_points_by_step = dict(sorted(kst_traj.points.items()))
        retval: List[ReactivePlannerState] = list()
        for step, point in ordered_points_by_step.items():
            retval.append(
                self.sample_c2p_kst(kst_state=point, mode=mode, time_step=step)
            )
        return retval



    def sample_c2p_kst(
            self,
            kst_state: Union[KSTState, KSTInput],
            mode: Literal['state', 'input'],
            time_step: int
    ) -> Union[ReactivePlannerState, InputState]:
        """
        Get Reactive planner state or input from kst state or input
        :param kst_state:
        :param mode:
        :param time_step:
        :return:
        """
        if mode == 'state':
            retval: ReactivePlannerState = ReactivePlannerState(
                time_step=time_step,
                position=np.asarray([kst_state.position_x, kst_state.position_y]),
                velocity=kst_state.velocity,
                acceleration=kst_state.acceleration,
                orientation=kst_state.heading,
                steering_angle=kst_state.steering_angle,
                yaw_rate=0
            )
        else:
            retval: InputState = InputState(
                steering_angle_speed=kst_state.steering_angle_velocity,
                acceleration=kst_state.jerk,
                time_step=time_step
            )
        return retval




    # --- DST ---
    def trajectory_p2c_dst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> DBTrajectory:
            return planner_traj

    def trajectory_c2p_dst(
            self,
            dst_traj: DBTrajectory,
            mode: Literal['state', 'input']
    ) -> Any:
        return dst_traj

    def sample_p2c_dst(
            self,
            planner_state: Any,
            mode: Literal['state', 'input']
    ) -> Union[DBState, DBInput]:
        return planner_state

    def sample_c2p_dst(
            self,
            dst_state: DBState,
            mode: Literal['state', 'input']
    ) -> Any:
        return dst_state

