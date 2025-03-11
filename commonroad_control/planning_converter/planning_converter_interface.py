from abc import ABC, abstractmethod
from typing import Union, Any, Literal

# own code base
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
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
            dst_factory: Union[DBSITFactory, Any] = DBSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        self._config: int = config
        self._kst_factory: Union[KSTSITFactory, Any] = kst_factory
        self._dst_factory: Union[DBSITFactory, Any] = dst_factory
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
            mode: Literal['state', 'input'],
            t_0: float,
            dt: float
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
            mode: Literal['state', 'input'],
            time_step: int
    ) -> Any:
        pass

    # DST
    @abstractmethod
    def trajectory_p2c_dst(
            self,
            planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> DBTrajectory:
            pass

    @abstractmethod
    def trajectory_c2p_dst(
            self,
            dst_traj: DBTrajectory,
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
            time_step: int,
            mode: Literal['state', 'input'],
    ) -> Any:
        pass