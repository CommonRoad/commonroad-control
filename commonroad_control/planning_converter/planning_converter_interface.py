from abc import ABC, abstractmethod
from typing import Union, Any, Literal

from commonroad_control.vehicle_dynamics.dynamic_bicycle.dst_trajectory import DSTTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.dst_sit_factory import DSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_input import KSTInput
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class PlanningConverterInterface(ABC):
    def __init__(
            self,
            config: int = 0,
            kst_factory: Union[KSTSITFactory, Any] = KSTSITFactory(),
            dst_factory: Union[DSTSITFactory, Any] = DSTSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        self._config: int = config
        self._kst_factory: Union[KSTSITFactory, Any] = kst_factory
        self._dst_factory: Union[DSTSITFactory, Any] = dst_factory
        self._vehicle_params: Union[BMW3seriesParams, Any] = vehicle_params


    @property
    def config(self) -> Any:
        return self._config

    @property
    def vehicle_params(self) -> Union[BMW3seriesParams, Any]:
        return self._vehicle_params

    @abstractmethod
    def trajectory_p2c_kst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> KSTTrajectory:
            pass

    # KST
    @abstractmethod
    def trajectory_c2p_kst(
            self,
            kst_traj: KSTTrajectory,
            mode: Literal['state', 'input']
    ) -> Any:
        pass

    @abstractmethod
    def sample_p2c_kst(
            self,
            planner_state: Any,
            mode: Literal['state', 'input']
    ) -> Union[KSTState, KSTInput]:
        pass

    @abstractmethod
    def sample_c2p_kst(
            self,
            kst_state: KSTState,
            mode: Literal['state', 'input']
    ) -> Any:
        pass

    # DST
    @abstractmethod
    def trajectory_p2c_dst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> DSTTrajectory:
            pass

    @abstractmethod
    def trajectory_c2p_dst(
            self,
            dst_traj: DSTTrajectory,
            mode: Literal['state', 'input']
    ) -> Any:
        pass

    @abstractmethod
    def sample_p2c_dst(
            self,
            planner_state: Any,
            mode: Literal['state', 'input']
    ) -> Union[DBState, DBInput]:
        pass

    @abstractmethod
    def sample_c2p_dst(
            self,
            dst_state: DBState,
            mode: Literal['state', 'input']
    ) -> Any:
        pass