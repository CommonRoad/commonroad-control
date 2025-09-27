from typing import Union, Any, Literal

from commonroad_control.planning_converter.planning_converter_interface import PlanningConverterInterface
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sit_factory import DBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class DummyPlanningConverter(PlanningConverterInterface):
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

    def __init__(
            self,
            config: int = 0,
            kb_factory: Union[KBSITFactory, Any] = KBSITFactory(),
            db_factory: Union[DBSITFactory, Any] = DBSITFactory(),
            vehicle_params: Union[BMW3seriesParams, Any] = BMW3seriesParams()
    ) -> None:
        super().__init__(
            config=config,
            kb_factory=kb_factory,
            db_factory=db_factory,
            vehicle_params=vehicle_params
        )


    def trajectory_p2c_kb(
            self,
            planner_traj: Any,
            mode: TrajectoryMode
    ) -> KBTrajectory:
            return planner_traj


    # kb
    def trajectory_c2p_kb(
            self,
            kb_traj: KBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        return kb_traj


    def sample_p2c_kb(
            self,
            planner_state: Any,
            mode: TrajectoryMode
    ) -> Union[KBState, KBInput]:
        return planner_state

    def sample_c2p_kb(
            self,
            kb_state: KBState,
            mode: TrajectoryMode
    ) -> Any:
        return kb_state

    # db
    def trajectory_p2c_db(
            self,
            planner_traj: Any,
            mode: TrajectoryMode
    ) -> DBTrajectory:
            return planner_traj

    def trajectory_c2p_db(
            self,
            db_traj: DBTrajectory,
            mode: TrajectoryMode
    ) -> Any:
        return db_traj

    def sample_p2c_db(
            self,
            planner_state: Any,
            mode: TrajectoryMode
    ) -> Union[DBState, DBInput]:
        return planner_state

    def sample_c2p_db(
            self,
            db_state: DBState,
            mode: TrajectoryMode
    ) -> Any:
        return db_state


