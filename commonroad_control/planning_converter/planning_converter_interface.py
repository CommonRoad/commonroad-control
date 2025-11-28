from abc import ABC, abstractmethod
from typing import Any, Union

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import (
    DBSIDTFactory,
)
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBState

# own code base
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import (
    DBTrajectory,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import (
    KBSITFactoryDisturbance,
)
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import (
    KBTrajectory,
)
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class PlanningConverterInterface(ABC):
    def __init__(
        self,
        kb_factory: Union[KBSITFactoryDisturbance, Any],
        db_factory: Union[DBSIDTFactory, Any],
        vehicle_params: Union[BMW3seriesParams, Any],
        *args,
        **kwargs,
    ) -> None:
        """
        Interface for the planner converter. The converter converts planner states to and from controller states.
        :param kb_factory: factory for kinematic single track states and inputs
        :param db_factory: factory for dynamic single track states and inputs
        :param vehicle_params: vehicle parameters
        """
        self._kb_factory: Union[KBSITFactoryDisturbance, DBSIDTFactory, Any] = (
            kb_factory
        )
        self._db_factory: Union[DBSIDTFactory, DBSIDTFactory, Any] = db_factory
        self._vehicle_params: Union[BMW3seriesParams, Any] = vehicle_params

    @property
    def kb_factory(self) -> Union[KBSITFactoryDisturbance, DBSIDTFactory, Any]:
        """
        :return: planner state factory
        """
        return self._kb_factory

    @property
    def db_factory(self) -> Union[DBSIDTFactory, DBSIDTFactory, Any]:
        """
        :return: controller state factory
        """
        return self._db_factory

    @property
    def vehicle_params(self) -> Union[BMW3seriesParams, Any]:
        """
        :return: vehicle parameters
        """
        return self._vehicle_params

    @abstractmethod
    def trajectory_p2c_kb(
        self, planner_traj: Any, mode: TrajectoryMode, t_0: float, dt: float
    ) -> KBTrajectory:
        """
        Generate Kinematic bycicle state or input trajectory from planner
        :param planner_traj: planner trajectory
        :param mode: state or input mode
        :param t_0: starting time
        :param dt: time step size
        :return: Kinematic bycicle trajectory
        """
        pass

    # kb
    @abstractmethod
    def trajectory_c2p_kb(self, kb_traj: KBTrajectory, mode: TrajectoryMode) -> Any:
        """
        Generate planner trajectory from kinematic bycicle trajectory
        :param kb_traj: Kinematic bycicle trajectory
        :param mode: state or input mode
        :return: planner trajectory
        """
        pass

    @abstractmethod
    def sample_p2c_kb(
        self,
        planner_state: Any,
        mode: TrajectoryMode,
    ) -> Union[KBState, KBInput]:
        """
        Convert planner state or input to kinematic bycicle state or input
        :param planner_state: planner state or input
        :param mode: state or input mode
        :return: Kinematic bycicle state or input
        """
        pass

    @abstractmethod
    def sample_c2p_kb(
        self, kb_state: KBState, mode: TrajectoryMode, time_step: int
    ) -> Any:
        """
        Converts kinematic bycicle state or input to planner state or input time step
        :param kb_state: kinematic bycicle state
        :param mode: state or input mode
        :param time_step: time step
        :return: planner state or input
        """
        pass

    # db
    @abstractmethod
    def trajectory_p2c_db(
        self, planner_traj: Any, mode: TrajectoryMode
    ) -> DBTrajectory:
        """
        Generate dynamic bycicle trajectory from planner trajectory
        :param planner_traj: planner trajectory
        :param mode: state or input mode
        :return: Dynamic bicycle trajectory
        """
        pass

    @abstractmethod
    def trajectory_c2p_db(self, db_traj: DBTrajectory, mode: TrajectoryMode) -> Any:
        """
        Convert planner state or input to dynamic state or input
        :param db_traj: Dynamic bicycle trajectory
        :param mode: state or input mode
        :return: Planner trajectory
        """
        pass

    @abstractmethod
    def sample_p2c_db(
        self, planner_state: Any, mode: TrajectoryMode
    ) -> Union[DBState, DBInput]:
        """
        Convert planner state or input to dynamic bicycle state or input
        :param planner_state: planner state or input
        :param mode: state or input mode
        :return: Dynamic bicycle state or input
        """
        pass

    @abstractmethod
    def sample_c2p_db(
        self,
        db_state: DBState,
        time_step: int,
        mode: TrajectoryMode,
    ) -> Any:
        """
        Convert dynamic bicycle state or input to planner state or input at time step
        :param db_state: Dynamic bicycle state or input
        :param time_step: time step
        :param mode: state or input mode
        :return: planner state or input
        """
        pass
