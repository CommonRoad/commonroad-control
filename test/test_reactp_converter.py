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
from rp_test import main as rpmain

from typing import List

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories


class TestReactivePlannerConverter(unittest.TestCase):


    def test_p2c_with_txt_data(self) -> None:
        """
        Tests conversion from and to reactive planner with txt data
        """
        input_file = Path(__file__).parents[0] / "reactive_planner_traj/input.txt"
        state_file = Path(__file__).parents[0] / "reactive_planner_traj/state.txt"

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
        kst_state_traj: KSTTrajectory = rpc.trajectory_p2c_kst(planner_traj=rp_states, mode='state')
        kst_input_traj: KSTTrajectory = rpc.trajectory_p2c_kst(planner_traj=rp_inputs, mode='input')

        rec_rp_state_traj: List[ReactivePlannerState] = rpc.trajectory_c2p_kst(kst_traj=kst_state_traj, mode='state')
        rec_rp_input_traj: List[InputState] = rpc.trajectory_c2p_kst(kst_traj=kst_input_traj, mode='input')


        if len(rec_rp_state_traj) != len(rp_states):
            raise ValueError(f"State and reconstructed conversion have mismatching length: rec={len(rec_rp_state_traj)}"
                             f"but original was {len(rp_states)}")

        if len(rec_rp_input_traj) != len(rp_inputs):
            raise ValueError(f"Input and reconstructed conversion have mismatching length: rec={len(rec_rp_input_traj)}"
                             f"but original was {len(rp_inputs)}")

        for idx, _ in enumerate(rec_rp_state_traj):
            if (
                    not np.isclose(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0]) or
                    rec_rp_state_traj[idx].position[1] != rp_states[idx].position[1] or
                    rec_rp_state_traj[idx].velocity != rp_states[idx].velocity or
                    rec_rp_state_traj[idx].steering_angle != rp_states[idx].steering_angle or
                    rec_rp_state_traj[idx].orientation != rp_states[idx].orientation or
                    rec_rp_state_traj[idx].time_step != rp_states[idx].time_step
            ):
                print(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0])
                raise ValueError(f"Mismatching states in reconstruction: "
                                 f"Reconstr. {rec_rp_state_traj[idx]}  --  Original {rp_states[idx]}")


        for idx, _ in enumerate(rec_rp_input_traj):
            if rec_rp_input_traj[idx] != rp_inputs[idx]:
                raise ValueError(f"Mismatching input in reconstruction: "
                                 f"Reconstr. {rec_rp_input_traj[idx]}  --  Original {rp_inputs[idx]}")




    def test_p2c_with_rp_integration(self) -> None:
        scenario_file = Path(__file__).parents[1] / "scenarios" / "ZAM_Over-1_1.xml"
        config_name = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / "ZAM_Over-1_1.yaml"
        filename = "ZAM_Over-1_1.xml"

        scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
        planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]
        # Build config object
        rp_config = ReactivePlannerConfiguration.load(config_name,
                                                      filename)
        rp_config.update(scenario=scenario, planning_problem=planning_problem)
        rp_config.planning_problem_set = planning_problem_set

        rp_states, rp_inputs = rpmain(config=rp_config)



        rpc = ReactivePlannerConverter()
        kst_state_traj: KSTTrajectory = rpc.trajectory_p2c_kst(planner_traj=rp_states, mode='state')
        kst_input_traj: KSTTrajectory = rpc.trajectory_p2c_kst(planner_traj=rp_inputs, mode='input')

        rec_rp_state_traj: List[ReactivePlannerState] = rpc.trajectory_c2p_kst(kst_traj=kst_state_traj, mode='state')
        rec_rp_input_traj: List[InputState] = rpc.trajectory_c2p_kst(kst_traj=kst_input_traj, mode='input')


        if len(rec_rp_state_traj) != len(rp_states):
            raise ValueError(f"State and reconstructed conversion have mismatching length: rec={len(rec_rp_state_traj)}"
                             f"but original was {len(rp_states)}")

        if len(rec_rp_input_traj) != len(rp_inputs):
            raise ValueError(f"Input and reconstructed conversion have mismatching length: rec={len(rec_rp_input_traj)}"
                             f"but original was {len(rp_inputs)}")

        for idx, _ in enumerate(rec_rp_state_traj):
            if (
                    rec_rp_state_traj[idx].position[0] != rp_states[idx].position[0] or
                    rec_rp_state_traj[idx].position[1] != rp_states[idx].position[1] or
                    rec_rp_state_traj[idx].velocity != rp_states[idx].velocity or
                    rec_rp_state_traj[idx].steering_angle != rp_states[idx].steering_angle or
                    rec_rp_state_traj[idx].orientation != rp_states[idx].orientation or
                    rec_rp_state_traj[idx].time_step != rp_states[idx].time_step
            ):
                print(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0])
                raise ValueError(f"Mismatching states in reconstruction: "
                                 f"Reconstr. {rec_rp_state_traj[idx]}  --  Original {rp_states[idx]}")


        for idx, _ in enumerate(rec_rp_input_traj):
            if rec_rp_input_traj[idx] != rp_inputs[idx]:
                raise ValueError(f"Mismatching input in reconstruction: "
                                 f"Reconstr. {rec_rp_input_traj[idx]}  --  Original {rp_inputs[idx]}")


        controller_trajectory = copy.deepcopy(kst_state_traj)
        for step, state in controller_trajectory.points.items():
            new_state = state
            new_state.position_y = state.position_y + 3
            controller_trajectory.points[step] = new_state

        visualize_trajectories(
            scenario=scenario,
            planning_problem=planning_problem,
            planner_trajectory=kst_state_traj,
            controller_trajectory=controller_trajectory,
            use_icons=True
        )