import unittest
import os
from pathlib import Path
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging
from mpc_example import main as mpc_main
from commonroad_control.util.cr_ci_utils import (
    CI_FULL_SCENARIO_TRIGGER,
    NUM_SCENARIO_SMALL_TEST
)


logger_global = configure_toolbox_logging(level=logging.INFO)

class TestMPCExample(unittest.TestCase):

    def test_mpc_example_on_scenarios(self) -> None:
        """
        Tests mpc example on the scenarios in test dir
        """
        not_working = [
            "C-DEU_B471-2_1",
            "ZAM_Over-1_1"
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
                img_save_path = Path(__file__).parents[1] / "output" / scenario_name
                mpc_main(
                    scenario_file=scenario_file,
                    scenario_name=scenario_name,
                    img_save_path=img_save_path,
                    planner_config_path=planner_config_path,
                    save_imgs=False,
                    create_scenario=False
                )

