import numpy as np
from dataclasses import dataclass

from commonroad.scenario.state import InitialState, CustomState

from commonroad_control.vehicle_dynamics.state_interface import StateInterface


# TODO should be an enum so no initialization?
@dataclass(frozen=True)
class KSTStateIndices:
    """
    Indices of the states.
    """
    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


# TODO: Move to python3.10 and use kw_only dataclass arg?
@dataclass
class KSTState(StateInterface):
    """
    State of the kinematic single track model.
    """
    dim: int = KSTStateIndices.dim
    position_x: float = None
    position_y: float = None
    velocity: float = None
    heading: float = None
    steering_angle: float = None

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """
        x = np.zeros((self.dim,))
        x[KSTStateIndices.position_x] = self.position_x
        x[KSTStateIndices.position_y] = self.position_y
        x[KSTStateIndices.velocity] = self.velocity
        x[KSTStateIndices.heading] = self.heading
        x[KSTStateIndices.steering_angle] = self.steering_angle

        return x

    def to_cr_initial_state(
            self,
            time_step: int
    ) -> InitialState:
        """
        Convert to cr initial state
        :param time_step: time step
        :return: cr InitialState
        """
        return InitialState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=self.velocity,
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=0,
            time_step=time_step
        )


    def to_cr_custom_state(
            self,
            time_step: int
    ) -> CustomState:
        """
        Convert to cr custom state
        :param time_step: time step
        :return: cr custom state
        """
        return CustomState(
            position=np.asarray([self.position_x, self.position_y]),
            velocity=self.velocity,
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=0,
            time_step=time_step
        )

