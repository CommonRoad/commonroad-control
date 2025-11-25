import unittest
from pathlib import Path
import os
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging
from pid_long_lat import main

logger_global = configure_toolbox_logging(level=logging.DEBUG)


class TestPIDLongLatExample(unittest.TestCase):


    def test_pid_long_lat_example(self) -> None:
        """
        Test pid long lat example script
        """

        not_working = [
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
                main(
                    scenario_file=scenario_file,
                    scenario_name=scenario_name,
                    img_save_path=img_save_path,
                    planner_config_path=planner_config_path,
                    save_imgs=False,
                    create_scenario=False
                )

                