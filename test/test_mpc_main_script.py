import unittest
import os
from pathlib import Path

from mpc_example import main as mpc_main


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

        for scenario_name_xml in [el for el in sorted(os.listdir(path_scenarios))]:
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

