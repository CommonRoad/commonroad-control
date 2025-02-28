import warnings
from abc import ABC, abstractmethod
from typing import Union, Any, Literal

from commonroad.prediction.prediction import TrajectoryPrediction

from commonroad_control.vehicle_dynamics.dynamic_bicycle.dst_sit_factory import DSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_sit_factory import KSTSITFactory
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_state import KSTState
from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams


class Planning2ControlConverter:
    # TODO decide if it needs a config? Otherwise methods static.
    # TODO: Aggregation makes sense?

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

    def dummy_trajectory_conversion_to_control(
            self,
            reactive_planner_traj: Any,
            mode: Literal['state', 'input']
    ) -> KSTTrajectory:
        if mode == 'state':
            print(f"state conversion")
        else:
            print(f"input conversion")
        warnings.warn(f"Dummy implementation")
        return reactive_planner_traj

    def dummy_trajectory_conversion_from_control(
            self,
            kst_trajectoy: KSTTrajectory,
            mode: Literal['state', 'input']
    ) -> Any:
        if mode == 'state':
            print(f"state conversion")
        else:
            print(f"input conversion")
        warnings.warn(f"Dummy implementation")
        return kst_trajectoy

    def dummy_sample_to_control(
            self,
            reactive_planner_state: Any,
            mode: Literal['state', 'input']
    ) -> KSTState:
        if mode == 'state':
            print(f"state conversion")
        else:
            print(f"input conversion")
        warnings.warn(f"Dummy implementation")
        return reactive_planner_state


    def dummy_sample_from_control(
            self,
            kst_state: KSTState,
            mode: Literal['state', 'input']
    ) -> Any:
        if mode == 'state':
            print(f"state conversion")
        else:
            print(f"input conversion")
        warnings.warn(f"Dummy implementation")
        return kst_state


    def kst_2_cr_reactive_planner(
            self,
            kst_traj: KSTTrajectory
    ) -> Any:
        pass

    def cr_qp_planner_2_kst(
            self,
            qp_planner_traj: Any
    ) -> KSTTrajectory:
        pass

    def kst_2_cr_qp_planner(
            self,
            kst_traj: KSTTrajectory
    ) -> Any:
        pass


