import copy
import random
import unittest
import ast
from pathlib import Path


import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_rp.state import ReactivePlannerState
from commonroad.scenario.state import InputState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from rp_test import main as rpmain

from typing import List

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories

from commonroad_control.control.pid.pid_controller import PIDController




def main() -> None:
    kst_traj, kst_input = execute_planner()
    number_control_cycles_per_planning_cycle: int = 1

    # TODO: add forward simulation etc. for controller

    for idx, state in enumerate(kst_traj.points):
        if idx == 0:
            continue










def execute_planner() -> KSTTrajectory:
    """
    Dummy loading precomputed Reactive Planner KST Trajectory
    :return: kst trajectory
    """
    input_file = Path(__file__).parents[0] / "test/reactive_planner_traj/input.txt"
    state_file = Path(__file__).parents[0] / "test/reactive_planner_traj/state.txt"
    with open(input_file, "r") as f:
        i = [ast.literal_eval(el) for el in f.readlines()]
        rp_inputs: List[InputState] = list()
        for idx, el in enumerate(i):
            rp_inputs.append(InputState(acceleration=el[0], steering_angle_speed=el[1], time_step=idx))

    with open(state_file, "r") as f:
        s = [ast.literal_eval(el.replace("array", '')) for el in f.readlines()]
        rp_states: List[ReactivePlannerState] = list()
        for idx, el in enumerate(s):
            rp_states.append(
                ReactivePlannerState(
                    time_step=idx,
                    position=np.asarray(el[0]),
                    steering_angle=el[1],
                    velocity=el[2],
                    orientation=el[3],
                    acceleration=el[4],
                    yaw_rate=el[5]
                )
            )
    rpc = ReactivePlannerConverter()
    return rpc.trajectory_p2c_kst(planner_traj=rp_states, mode='state'), rpc.trajectory_p2c_kst(planner_traj=rp_states, mode='input')



if __name__ == "__main__":
    main()