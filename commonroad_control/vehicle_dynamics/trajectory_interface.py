import math
from dataclasses import dataclass
from abc import ABC, abstractmethod
from typing import Any, Union, Dict, Optional, Tuple, Literal


@dataclass
class TrajectoryInterface(ABC):
    # TODO Move some stuff to properties for access control as frozen does not work in our case?
    states: Dict[int, Any]
    delta_t: float
    mode: Literal['state', 'input']
    t_0: float = 0
    t_final: Optional[float] = None
    initial_state: Optional[Any] = None
    final_state: Optional[Any] = None
    dim: Optional[int] = None

    def __post_init__(self):
        self.sanity_check()
        self.dim = self.states[0].dim
        self.initial_state = self.states[min(self.states.keys())]
        self.final_state = self.states[max(self.states.keys())]
        self.t_final = self.t_0 + len(self.states.keys()) * self.delta_t


    def sanity_check(self) -> None:
        """
        Sanity check
        """
        if len(self.states.keys()) == 0:
            raise ValueError(f"states must contain more than 0 values")
        if None in self.states.values():
            raise ValueError(f"states must not contain None")
        d = self.states[0].dim
        for state in self.states.values():
            if state.dim != d:
                raise ValueError("states have varying dimension")

    def get_state_at_step(
            self,
            step: int
    ) -> Optional[Any]:
        """
        Returns State at step or None if not existing
        :param step: time step
        :return: Returns State at step or None if not existing
        """
        return self.states[step] if step in self.states.keys() else None


    def get_states_before_and_after_time(
            self,
            time: float
    ) -> Tuple[Any, Any, int, int]:
        """
        Finds states before and after a given time steps
        :param time: time
        :return: state_before, state_after, idx_before, idx_after
        """
        if time < self.t_0:
            raise ValueError(f"time {time} is before trajectory start {self.t_0}")
        idx_lower: int = math.floor((time - self.t_0) / self.delta_t)
        idx_upper: int = math.ceil((time - self.t_0) / self.delta_t)

        return self.states[idx_lower], self.states[idx_upper], idx_lower, idx_upper

    @abstractmethod
    def get_interpolated_state_at_time(
            self,
            time: float
    ) -> Union[Any]:
        """
        Compute interpolated state at time
        :param time: time of step
        :return: State
        """
        pass
