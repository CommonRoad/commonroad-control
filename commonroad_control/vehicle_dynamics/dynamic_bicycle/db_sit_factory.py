from typing import Union

import numpy as np

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input import DBInput, DBInputIndices
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state import DBStateIndices, DBState
from commonroad_control.vehicle_dynamics.sit_factory_interface import StateInputTrajectoryFactoryInterface


class DBSITFactory(StateInputTrajectoryFactoryInterface):
    """
    Dynamic bicycle model factory for state, input, and trajectory.
    """
    def state_from_numpy_array(
            self,
            x_np: np.array,
    ) -> Union['DBState']:
        """
        Set values of class from a given array.
        :param x_np: state - array of dimension (dim,)
        :return: dst state
        """
        if x_np.shape[0] != DBStateIndices.dim:
            raise ValueError(f'Dimension {x_np.shape[0]} does not match')
        if x_np.size > 1:
            raise ValueError(f"size of np_array should be (dim,1) but is {x_np}")

        return DBState(
            position_x=x_np[DBStateIndices.position_x],
            position_y=x_np[DBStateIndices.position_y],
            velocity_long=x_np[DBStateIndices.velocity_long],
            velocity_lat=x_np[DBStateIndices.velocity_lat],
            acceleration=x_np[DBStateIndices.acceleration],
            heading=x_np[DBStateIndices.heading],
            yaw_rate=x_np[DBStateIndices.yaw_rate],
            steering_angle=x_np[DBStateIndices.steering_angle]
        )

    def input_from_numpy_array(
            self,
            u_np: np.array
    ) -> Union['DBInput']:
        """
        Set values from a given array.
        :param u_np: input vector - array of dimension (self.dim,)
        :return: DST input
        """
        if u_np.size > 1:
            raise ValueError(f"size of array should be (dim,) but is {u_np}")
        if u_np.shape[0] != DBStateIndices.dim:
            raise ValueError(f"input should be ({DBInputIndices.dim},) but is {u_np}")

        return DBInput(
            jerk=u_np[DBInputIndices.jerk],
            steering_angle_velocity=u_np[DBInputIndices.steering_angle_velocity]
        )




