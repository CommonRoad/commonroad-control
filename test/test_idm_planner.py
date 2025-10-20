import unittest
from pathlib import Path
import os
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_idm_planner.idm_planner import IDMPlanner
from commonroad_idm_planner.idm_input import IDMInput
from commonroad_idm_planner.idm_state import IDMState
from commonroad_idm_planner.idm_trajectory import IDMTrajectory
from commonroad_idm_planner.idm_easy_api import solve_planning_problem_and_get_state_and_input_trajectory


class TestIDMPlanner(unittest.TestCase):

    def test_idm_planner(self) -> None:
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
                scenario, planning_problem_set = CommonRoadFileReader(scenario_file).open()
                planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]

                state_traj, input_traj = solve_planning_problem_and_get_state_and_input_trajectory(
                    scenario=scenario,
                    planning_problem=planning_problem
                )

                asdf = 3