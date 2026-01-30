import unittest
from pathlib import Path
import os
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import pid_with_lookahead_for_reactive_planner
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner
from commonroad_control.util.cr_ci_utils import (
    CI_FULL_SCENARIO_TRIGGER,
    NUM_SCENARIO_SMALL_TEST
)

logger_global = configure_toolbox_logging(level=logging.INFO)


class TestCREasyAPI(unittest.TestCase):


    def test_pid_for_reactive(self) -> None:
        """
        Test pid long lat example script
        """
        not_working = [
        ]

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        sorted_scenarios = sorted(os.listdir(path_scenarios))
        if not os.getenv(CI_FULL_SCENARIO_TRIGGER):
            sorted_scenarios = sorted_scenarios[:min(NUM_SCENARIO_SMALL_TEST, len(sorted_scenarios))]

        for scenario_name_xml in sorted_scenarios:
            scenario_name = scenario_name_xml.split(".")[0]
            scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")

            if not os.path.isfile(scenario_file) or scenario_name in not_working:
                continue

            with self.subTest(msg=f"testing scenario {scenario_name}"):
                planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
                    scenario_name + ".yaml")
                scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
                planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

                print(f"solving scenario {str(scenario.scenario_id)}")

                # run planner
                print("run planner")
                rp_states, rp_inputs = run_reactive_planner(
                    scenario=scenario,
                    scenario_xml_file_name=str(scenario_name + ".xml"),
                    planning_problem=planning_problem,
                    planning_problem_set=planning_problem_set,
                    reactive_planner_config_path=planner_config_path
                )

                print("run controller")
                noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner(
                    scenario=scenario,
                    planning_problem=planning_problem,
                    reactive_planner_state_trajectory=rp_states,
                    reactive_planner_input_trajectory=rp_inputs,
                    visualize_scenario=False,
                    visualize_control=False,
                    save_imgs=False,
                    img_saving_path=None
                )



    def test_pid_for_reactive_visualization(self) -> None:
        """
        Test pid long lat example script visualization
        """
        scenario_name = "ITA_Foggia-6_1_T-1"
        scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")


        with self.subTest(msg=f"testing scenario {scenario_name}"):
            planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
                scenario_name + ".yaml")
            scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
            planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

            print(f"solving scenario {str(scenario.scenario_id)}")

            # run planner
            print("run planner")
            rp_states, rp_inputs = run_reactive_planner(
                scenario=scenario,
                scenario_xml_file_name=str(scenario_name + ".xml"),
                planning_problem=planning_problem,
                planning_problem_set=planning_problem_set,
                reactive_planner_config_path=planner_config_path
            )

            print("run controller")
            noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner(
                scenario=scenario,
                planning_problem=planning_problem,
                reactive_planner_state_trajectory=rp_states,
                reactive_planner_input_trajectory=rp_inputs,
                visualize_scenario=True,
                visualize_control=True,
                save_imgs=False,
                img_saving_path=None
            )

