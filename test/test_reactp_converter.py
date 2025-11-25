import unittest
import ast
from pathlib import Path
import matplotlib.pyplot as plt
import logging


import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_rp.state import ReactivePlannerState
from commonroad.scenario.state import InputState
from commonroad_rp.utility.config import ReactivePlannerConfiguration

from commonroad_control.planning_converter.reactive_planner_converter import ReactivePlannerConverter
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory import DBTrajectory
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from typing import List

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.util.visualization.visualize_trajectories import visualize_trajectories
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

logger_global = configure_toolbox_logging(level=logging.DEBUG)

class TestReactivePlannerConverter(unittest.TestCase):


    def test_p2c_with_txt_data(self) -> None:
        """
        Tests conversion from and to reactive planner with txt data
        """
        input_file = Path(__file__).parents[0] / "reactive_planner_traj/ZAM_Over-1_1/input.txt"
        state_file = Path(__file__).parents[0] / "reactive_planner_traj/ZAM_Over-1_1/state.txt"


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


        # -- Subtest for KST Conversion --
        with self.subTest(msg=f"KST Conversion P2C and C2P"):
            rpc = ReactivePlannerConverter()
            kst_state_traj: KBTrajectory = rpc.trajectory_p2c_kb(planner_traj=rp_states, mode=TrajectoryMode.State)
            kst_input_traj: KBTrajectory = rpc.trajectory_p2c_kb(planner_traj=rp_inputs, mode=TrajectoryMode.Input)

            rec_rp_state_traj: List[ReactivePlannerState] = rpc.trajectory_c2p_kb(kb_traj=kst_state_traj, mode=TrajectoryMode.State)
            rec_rp_input_traj: List[InputState] = rpc.trajectory_c2p_kb(kb_traj=kst_input_traj, mode=TrajectoryMode.Input)


            if len(rec_rp_state_traj) != len(rp_states):
                raise ValueError(f"State and reconstructed conversion have mismatching length: rec={len(rec_rp_state_traj)}"
                                 f"but original was {len(rp_states)}")

            if len(rec_rp_input_traj) != len(rp_inputs):
                raise ValueError(f"Input and reconstructed conversion have mismatching length: rec={len(rec_rp_input_traj)}"
                                 f"but original was {len(rp_inputs)}")

            for idx, _ in enumerate(rec_rp_state_traj):
                if (
                        not np.isclose(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0]) or
                        not np.isclose(rec_rp_state_traj[idx].position[1], rp_states[idx].position[1]) or
                        not np.isclose(rec_rp_state_traj[idx].velocity, rp_states[idx].velocity) or
                        not np.isclose(rec_rp_state_traj[idx].steering_angle, rp_states[idx].steering_angle) or
                        not np.isclose(rec_rp_state_traj[idx].orientation, rp_states[idx].orientation) or
                        not np.isclose(rec_rp_state_traj[idx].time_step, rp_states[idx].time_step)
                ):
                    print(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0])
                    plt.figure()
                    plt.title("Conversion Error in KST Conversion P2C and C2P")
                    plt.plot(rp_states[idx].position[0], rp_states[idx].position[1], 'x', label="planner")
                    plt.plot(kst_state_traj.points[idx].position_x, kst_state_traj.points[idx].position_y, 'D',
                             label="KST")
                    plt.plot(rec_rp_state_traj[idx].position[0], rec_rp_state_traj[idx].position[1], 'o', label="reconstructed")
                    plt.legend()
                    plt.show()

                    raise ValueError(f"Mismatching states in reconstruction at idx={idx}: "
                                     f"Reconstr. {rec_rp_state_traj[idx]}  --  Original {rp_states[idx]}")


            for idx, _ in enumerate(rec_rp_input_traj):
                if rec_rp_input_traj[idx] != rp_inputs[idx]:
                    raise ValueError(f"Mismatching input in reconstruction at idx={idx}: "
                                     f"Reconstr. {rec_rp_input_traj[idx]}  --  Original {rp_inputs[idx]}")


        # -- Subtest for DST conversion --
        with self.subTest(msg="DST Conversion planning to control"):
            rpc = ReactivePlannerConverter()
            dst_state_traj: DBTrajectory = rpc.trajectory_p2c_db(planner_traj=rp_states, mode=TrajectoryMode.State)
            dst_input_traj: DBTrajectory = rpc.trajectory_p2c_db(planner_traj=rp_inputs, mode=TrajectoryMode.Input)

            reconverter_initial_state = rpc.sample_c2p_db(
                dst_state_traj.initial_point,
                mode=TrajectoryMode.State,
                time_step=min(dst_state_traj.steps)
            )
            reconverter_initial_state.acceleration = 0.0

            reconverter_initial_input = rpc.sample_c2p_db(
                dst_input_traj.initial_point,
                mode=TrajectoryMode.Input,
                time_step=min(dst_input_traj.steps)
            )


            if rp_states[0] != reconverter_initial_state:
                raise ValueError(
                    f"Initial states {rp_states[0]} and reconversion {reconverter_initial_state} do not match"
                )

            if (rp_inputs[0] != reconverter_initial_input):
                raise ValueError(
                    f"Initial inputs {rp_inputs[0]} and reconversion {reconverter_initial_input} do not match"
                )

        with self.subTest(msg="KST Over DST Visualization"):
            scenario_file = Path(__file__).parents[1] / "scenarios" / "ZAM_Over-1_1.xml"
            scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

            visualize_trajectories(
                scenario=scenario,
                planning_problem=planning_problem,
                planner_trajectory=kst_state_traj,
                controller_trajectory=dst_state_traj
            )


    def test_p2c_with_rp_integration(self) -> None:
        """
        Tests planning-to-control integration with reactive planner
        """
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

        planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
            filename[:-4] + ".yaml")

        print(f"solving scenario {str(scenario.scenario_id)}")

        # run planner
        print("run planner")
        rp_states, rp_inputs = run_reactive_planner(
            scenario=scenario,
            scenario_xml_file_name=str(filename[:-4] + ".xml"),
            planning_problem=planning_problem,
            planning_problem_set=planning_problem_set,
            reactive_planner_config_path=planner_config_path
        )



        rpc = ReactivePlannerConverter()
        kst_state_traj: KBTrajectory = rpc.trajectory_p2c_kb(planner_traj=rp_states, mode=TrajectoryMode.State)
        kst_input_traj: KBTrajectory = rpc.trajectory_p2c_kb(planner_traj=rp_inputs, mode=TrajectoryMode.Input)

        rec_rp_state_traj: List[ReactivePlannerState] = rpc.trajectory_c2p_kb(
            kb_traj=kst_state_traj,
            mode=TrajectoryMode.State
        )
        rec_rp_input_traj: List[InputState] = rpc.trajectory_c2p_kb(kb_traj=kst_input_traj, mode=TrajectoryMode.Input)


        if len(rec_rp_state_traj) != len(rp_states):
            raise ValueError(f"State and reconstructed conversion have mismatching length: rec={len(rec_rp_state_traj)}"
                             f"but original was {len(rp_states)}")

        if len(rec_rp_input_traj) != len(rp_inputs):
            raise ValueError(f"Input and reconstructed conversion have mismatching length: rec={len(rec_rp_input_traj)}"
                             f"but original was {len(rp_inputs)}")

        for idx, _ in enumerate(rec_rp_state_traj):
            if (
                    not np.isclose(rec_rp_state_traj[idx].position[0], rp_states[idx].position[0]) or
                    not np.isclose(rec_rp_state_traj[idx].position[1], rp_states[idx].position[1]) or
                    not np.isclose(rec_rp_state_traj[idx].velocity, rp_states[idx].velocity) or
                    not np.isclose(rec_rp_state_traj[idx].steering_angle, rp_states[idx].steering_angle) or
                    not np.isclose(rec_rp_state_traj[idx].orientation, rp_states[idx].orientation) or
                    not np.isclose(rec_rp_state_traj[idx].time_step, rp_states[idx].time_step)
            ):
                plt.figure()
                plt.title(f"Conversion Error Reactive Planner Integration at idx={idx}")
                plt.plot(rp_states[idx].position[0], rp_states[idx].position[1], 'x',label="planner")
                plt.plot(kst_state_traj.points[idx].position_x, kst_state_traj.points[idx].position_y, 'D',
                         label="KST")
                plt.plot(rec_rp_state_traj[idx].position[0], rec_rp_state_traj[idx].position[1], 'o',
                         label="reconstructed")
                plt.legend()
                plt.show()
                raise ValueError(f"Mismatching states in reconstruction at idx={idx}: "
                                 f"Reconstr. {rec_rp_state_traj[idx]}  --  Original {rp_states[idx]}")


        for idx, _ in enumerate(rec_rp_input_traj):
            if rec_rp_input_traj[idx] != rp_inputs[idx]:
                raise ValueError(f"Mismatching input in reconstruction: "
                                 f"Reconstr. {rec_rp_input_traj[idx]}  --  Original {rp_inputs[idx]}")
