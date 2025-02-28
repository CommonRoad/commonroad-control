import warnings
from abc import ABC, abstractmethod
from typing import Union, Any, Literal

from commonroad.prediction.prediction import TrajectoryPrediction

from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
from commonroad_control.vehicle_dynamics.dst_trajectory import DSTTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dst_sit_factory import DSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class DummyPlanningConverter(PlanningConverterInterface):
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

    def __init__(
            self,
            config: int = 0,
            kst_factory: Union[KSTSITFactory, Any] = KSTSITFactory(),
            dst_factory: Union[DSTSITFactory, Any] = DSTSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        super().__init__(
            config=config,
            kst_factory=kst_factory,
            dst_factory=dst_factory,
            vehicle_params=vehicle_params
        )


    def trajectory_p2c_kst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> KSTTrajectory:
            return planner_traj


    # KST
    def trajectory_c2p_kst(
            self,
            kst_traj: KSTTrajectory,
            mode: Literal['state', 'input']
    ) -> Any:
        return kst_traj


    def sample_p2c_kst(
            self,
            planner_state: Any,
            mode: Literal['state', 'input']
    ) -> Union[KSTState, KSTInput]:
        return planner_state

    def sample_c2p_kst(
            self,
            kst_state: KSTState,
            mode: Literal['state', 'input']
    ) -> Any:
        return kst_state

    # DST
    def trajectory_p2c_dst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> DSTTrajectory:
            return planner_traj

    def trajectory_c2p_dst(
            self,
            dst_traj: DSTTrajectory,
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


