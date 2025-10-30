import numpy as np
from math import sqrt
from dataclasses import dataclass

from commonroad.scenario.state import InitialState, CustomState

from commonroad_control.util.conversion_util import compute_total_velocity_from_components
from commonroad_control.vehicle_dynamics.state_interface import StateInterface, StateInterfaceIndex


@dataclass(frozen=True)
class DBStateIndices(StateInterfaceIndex):
    """
    Indices of the states.
    """
    dim: int = 7
    position_x: int = 0
    position_y: int = 1
    velocity_long: int = 2
    velocity_lat: int = 3
    heading: int = 4
    yaw_rate: int = 5
    steering_angle: int = 6


@dataclass
class DBState(StateInterface):
    """
    State of the dynamic bicycle model
    """
    position_x: float = None
    position_y: float = None
    velocity_long: float = None
    velocity_lat: float = None
    heading: float = None
    yaw_rate: float = None
    steering_angle: float = None

    @property
    def dim(self):
        return DBStateIndices.dim

    @property
    def velocity(self):
        return sqrt(self.velocity_long**2 + self.velocity_lat**2)

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (dim,)
        """

        x_np = np.zeros((self.dim,))
        x_np[DBStateIndices.position_x] = self.position_x
        x_np[DBStateIndices.position_y] = self.position_y
        x_np[DBStateIndices.velocity_long] = self.velocity_long
        x_np[DBStateIndices.velocity_lat] = self.velocity_lat
        x_np[DBStateIndices.heading] = self.heading
        x_np[DBStateIndices.yaw_rate] = self.yaw_rate
        x_np[DBStateIndices.steering_angle] = self.steering_angle

        return x_np

    # TODO: Add conversion of slip angle etc.
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
            velocity=compute_total_velocity_from_components(
                v_long=self.velocity_long,
                v_lat=self.velocity_lat
            ),
            orientation=self.heading,
            acceleration=0,
            yaw_rate=self.yaw_rate,
            slip_angle=0,
            time_step=time_step
        )

    # TODO: Add conversion of slip angle etc.
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
            velocity=compute_total_velocity_from_components(
                v_long=self.velocity_long,
                v_lat=self.velocity_lat
            ),
            orientation=self.heading,
            acceleration=0,
            yaw_rate=0,
            slip_angle=0,
            time_step=time_step
        )
