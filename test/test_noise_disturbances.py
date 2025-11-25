import unittest
from pathlib import Path
import os

from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import pid_with_lookahead_for_reactive_planner, \
    pid_with_lookahead_for_reactive_planner_no_uncertainty
from commonroad.common.file_reader import CommonRoadFileReader

from commonroad_control.simulation.uncertainty_model.gaussian_distribution import GaussianDistribution
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner



class TestNoiseDisturbance(unittest.TestCase):


    def test_no_uncertainty(self) -> None:
        """
        Test no uncertainty
        """
        not_working = [
        ]

        path_scenarios = Path(__file__).parents[1] / "scenarios"

        for scenario_name_xml in [el for el in sorted(os.listdir(path_scenarios))]:
            scenario_name = scenario_name_xml.split(".")[0]
            scenario_file = Path(__file__).parents[1] / "scenarios" / str(scenario_name + ".xml")

            if not os.path.isfile(scenario_file) or scenario_name in not_working:
                continue

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

            with self.subTest(msg=f"testing no uncertainty scenario {scenario_name}"):
                noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner_no_uncertainty(
                    scenario=scenario,
                    planning_problem=planning_problem,
                    reactive_planner_state_trajectory=rp_states,
                    reactive_planner_input_trajectory=rp_inputs,
                    visualize_scenario=False,
                    visualize_control=False,
                    save_imgs=False,
                    img_saving_path=None
                )

            with self.subTest(msg=f"testing gaussian scenario {scenario_name}"):
                noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner(
                    scenario=scenario,
                    planning_problem=planning_problem,
                    reactive_planner_state_trajectory=rp_states,
                    reactive_planner_input_trajectory=rp_inputs,
                    noise_model_typename=GaussianDistribution,
                    disturbance_model_typename=GaussianDistribution,
                    visualize_scenario=False,
                    visualize_control=False,
                    save_imgs=False,
                    img_saving_path=None
                )



