import copy
import math
import unittest
from pathlib import Path
import os
import logging

from closed_planning_control_loop_example import main
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

logger_global = configure_toolbox_logging(level=logging.DEBUG)

class TestPlanningControlLoop(unittest.TestCase):


    def test_pc_loop_for_reactive(self) -> None:
        """
        Test planner and control loop for reactive planner.
        """
        scenario_config = {
            "ITA_Foggia-6_1_T-1": [17, 6],
            # "ZAM_Tjunction-1_42_T-1": [30, 6],
            "C-DEU_B471-2_1": [30, 6],
            "DEU_AachenFrankenburg-1_2621353_T-21698": [20, 6],
            "DEU_Backnang-9_1_T-1": [35, 6],
            "DEU_Ibbenbueren-10_2_T-1": [30, 6],
            # "DEU_Salzwedel-107_1_T-7": [17, 6],
            "ZAM_Tutorial_Urban-3_2": [25, 6],
            "ZAM_Over-1_1": [26, 6],
        }

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        for scenario_name_xml in [el for el in sorted(os.listdir(path_scenarios))]:
            scenario_name = scenario_name_xml.split(".")[0]

            if scenario_name not in scenario_config.keys():
                continue

            with self.subTest(f"Closed-loop test scenario {scenario_name_xml}"):
                scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")
                planner_config_path = Path(__file__).parents[1] / "scenarios" / "reactive_planner_configs" / str(
                    scenario_name + ".yaml")

                planning_cycle_steps: int = scenario_config[scenario_name][0]
                max_replanning_iterations: int = scenario_config[scenario_name][1]

                main(
                    scenario_xml=scenario_file,
                    planner_config_yaml=planner_config_path,
                    planning_cycle_steps=planning_cycle_steps,
                    max_replanning_iterations=max_replanning_iterations,
                    img_save_path=None,
                    create_scenario=False,
                    save_imgs=False
                )


