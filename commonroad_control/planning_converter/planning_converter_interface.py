from abc import ABC, abstractmethod
from typing import Union, Any, Literal

# own code base
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactoryDisturbance
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class PlanningConverterInterface(ABC):
    def __init__(
            self,
            config: int = 0,
            kb_factory: Union[KBSITFactoryDisturbance, Any] = KBSITFactoryDisturbance(),
            db_factory: Union[DBSIDTFactory, Any] = DBSIDTFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        self._config: int = config
        self._kb_factory: Union[KBSITFactoryDisturbance, Any] = kb_factory
        self._db_factory: Union[DBSIDTFactory, Any] = db_factory
        self._vehicle_params: Union[BMW3seriesParams, Any] = vehicle_params


    @property
    def config(self) -> Any:
        return self._config

    @property
    def vehicle_params(self) -> Union[BMW3seriesParams, Any]:
        return self._vehicle_params

    @abstractmethod
    def trajectory_p2c_kb(
            self,
            planner_traj: Any,
            mode: TrajectoryMode,
            t_0: float,
            dt: float
    ) -> KBTrajectory:
        pass
    # kb
    @abstractmethod
    def trajectory_c2p_kb(
            self,
            kb_traj: KBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        pass

    @abstractmethod
    def sample_p2c_kb(
            self,
            planner_state: Any,
            mode: TrajectoryMode,
    ) -> Union[KBState, KBInput]:
        pass

    @abstractmethod
    def sample_c2p_kb(
            self,
            kb_state: KBState,
            mode: TrajectoryMode,
            time_step: int
    ) -> Any:
        pass

    # db
    @abstractmethod
    def trajectory_p2c_db(
            self,
            planner_traj: Any,
            mode: TrajectoryMode
    ) -> DBTrajectory:
            pass

    @abstractmethod
    def trajectory_c2p_db(
            self,
            db_traj: DBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        pass

    @abstractmethod
    def sample_p2c_db(
            self,
            planner_state: Any,
            mode: TrajectoryMode
    ) -> Union[DBState, DBInput]:
        pass

    @abstractmethod
    def sample_c2p_db(
            self,
            db_state: DBState,
            time_step: int,
            mode: TrajectoryMode,
    ) -> Any:
        pass