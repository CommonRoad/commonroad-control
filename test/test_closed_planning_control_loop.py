import unittest
from pathlib import Path
import os
import logging

from closed_planning_control_loop_example import main
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging
from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import (
    pid_with_lookahead_for_reactive_planner,
)
from commonroad_control.cr_control_easy_api.mpc_for_dedicated_planner import (
    mpc_for_reactive_planner
)
from commonroad_control.util.cr_ci_utils import (
    CI_FULL_SCENARIO_TRIGGER,
    NUM_SCENARIO_SMALL_TEST
)

logger_global = configure_toolbox_logging(level=logging.INFO)

class TestPlanningControlLoop(unittest.TestCase):


    def test_pid_pc_loop_for_reactive(self) -> None:
        """
        Test planner and control loop for reactive planner with PID controller.
        """
        scenario_config_pid = {
            "ITA_Foggia-6_1_T-1": [17, 6],
            # "ZAM_Tjunction-1_42_T-1": [30, 6],
            "C-DEU_B471-2_1": [30, 6],
            # "DEU_AachenFrankenburg-1_2621353_T-21698": [20, 6],
            "DEU_Backnang-9_1_T-1": [35, 6],
            "DEU_Ibbenbueren-10_2_T-1": [30, 6],
            # "DEU_Salzwedel-107_1_T-7": [17, 6],
            "ZAM_Tutorial_Urban-3_2": [25, 6],
            "ZAM_Over-1_1": [26, 6],
        }

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        sorted_scenarios = sorted(os.listdir(path_scenarios))
        if not os.getenv(CI_FULL_SCENARIO_TRIGGER):
            sorted_scenarios = sorted_scenarios[:min(NUM_SCENARIO_SMALL_TEST, len(sorted_scenarios))]

        for scenario_name_xml in sorted_scenarios:
            scenario_name = scenario_name_xml.split(".")[0]

            if scenario_name not in scenario_config_pid.keys():
                continue

            with self.subTest(f"Closed-loop PID test scenario {scenario_name_xml}"):
                scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")
                planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
                    scenario_name + ".yaml")

                planning_cycle_steps: int = scenario_config_pid[scenario_name][0]
                max_replanning_iterations: int = scenario_config_pid[scenario_name][1]

                main(
                    scenario_xml=scenario_file,
                    planner_config_yaml=planner_config_path,
                    controller_func=pid_with_lookahead_for_reactive_planner,
                    planning_cycle_steps=planning_cycle_steps,
                    max_replanning_iterations=max_replanning_iterations,
                    img_save_path=None,
                    create_scenario=False,
                    save_imgs=False
                )


    def test_mpc_pc_loop_for_reactive(self) -> None:
        """
        Test planner and control loop for reactive planner with MPC.
        """
        scenario_config_mpc = {
            "ITA_Foggia-6_1_T-1": [17, 6],
            # "ZAM_Tjunction-1_42_T-1": [30, 6],
            # "C-DEU_B471-2_1": [30, 6],
            # "DEU_AachenFrankenburg-1_2621353_T-21698": [20, 6],
            "DEU_Backnang-9_1_T-1": [35, 6],
            "DEU_Ibbenbueren-10_2_T-1": [30, 6],
            # "DEU_Salzwedel-107_1_T-7": [17, 6],
            # "ZAM_Tutorial_Urban-3_2": [25, 6],
            # "ZAM_Over-1_1": [26, 6],
        }

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        sorted_scenarios = sorted(os.listdir(path_scenarios))
        if not os.getenv(CI_FULL_SCENARIO_TRIGGER):
            sorted_scenarios = sorted_scenarios[:min(NUM_SCENARIO_SMALL_TEST, len(sorted_scenarios))]

        for scenario_name_xml in sorted_scenarios:
            scenario_name = scenario_name_xml.split(".")[0]

            if scenario_name not in scenario_config_mpc.keys():
                continue

            with self.subTest(f"Closed-loop MPC test scenario {scenario_name_xml}"):
                scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")
                planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
                    scenario_name + ".yaml")

                planning_cycle_steps: int = scenario_config_mpc[scenario_name][0]
                max_replanning_iterations: int = scenario_config_mpc[scenario_name][1]

                main(
                    scenario_xml=scenario_file,
                    planner_config_yaml=planner_config_path,
                    controller_func=mpc_for_reactive_planner,
                    planning_cycle_steps=planning_cycle_steps,
                    max_replanning_iterations=max_replanning_iterations,
                    img_save_path=None,
                    create_scenario=False,
                    save_imgs=False
                )