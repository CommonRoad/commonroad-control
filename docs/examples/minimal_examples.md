# Minimal examples
Here are some minimal examples for running different controllers with a planner. Please refer to the Easy API of 
CommonRoad Control for further documentation.

## PID with reactive planner
You can run the Long-Lat separated PID controller with the CommonRoad Reactive Planner using: 

```Python3
from pathlib import Path
import os
import logging

from commonroad_control.util.cr_logging_utils import configure_toolbox_logging

from commonroad_control.cr_control_easy_api.pid_for_dedicated_planner import pid_with_lookahead_for_reactive_planner
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner


logger_global = configure_toolbox_logging(level=logging.INFO)

# Read in scenario
scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/SCENARIO.xml").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]


# run planner. Either use this wrapper or the actual reactive planner.
rp_states, rp_inputs = run_reactive_planner(
    scenario=scenario,
    scenario_xml_file_name=str("SCENARIO_NAME" + ".xml"),
    planning_problem=planning_problem,
    planning_problem_set=planning_problem_set,
    reactive_planner_config_path="PATH/TO/REACTIVE/PLANNER/CONFIG.yaml"
)

# run controller
noisy_traj, disturbed_traj, input_traj = pid_with_lookahead_for_reactive_planner(
    scenario=scenario,
    planning_problem=planning_problem,
    reactive_planner_state_trajectory=rp_states,
    reactive_planner_input_trajectory=rp_inputs,
)
```

If you want to have no noise and disturbances, you can use `pid_with_lookahead_for_reactive_planner_no_uncertainty(...)` instead
of `pid_with_lookahead_for_reactive_planner(...)` using the same arguments.


## MPC with reactive planner
You can run the combined MPC controller with the CommonRoad Reactive Planner using: 

```Python3
from pathlib import Path
import os
import logging

from commonroad_control.cr_control_easy_api.mpc_for_dedicated_planner import mpc_for_reactive_planner
from commonroad_control.util.cr_logging_utils import configure_toolbox_logging
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_control.util.planner_execution_util.reactive_planner_exec_util import run_reactive_planner

logger_global = configure_toolbox_logging(level=logging.INFO)

# Read in scenario
scenario, planning_problem_set = CommonRoadFileReader("PATH/TO/SCENARIO.xml").open()
planning_problem = list(planning_problem_set.planning_problem_dict.values())[0]


# run planner. Either use this wrapper or the actual reactive planner.
rp_states, rp_inputs = run_reactive_planner(
    scenario=scenario,
    scenario_xml_file_name=str("SCENARIO_NAME" + ".xml"),
    planning_problem=planning_problem,
    planning_problem_set=planning_problem_set,
    reactive_planner_config_path="PATH/TO/REACTIVE/PLANNER/CONFIG.yaml"
)

# run controller 
noisy_traj, disturbed_traj, input_traj = mpc_for_reactive_planner(
    scenario=scenario,
    planning_problem=planning_problem,
    reactive_planner_state_trajectory=rp_states,
    reactive_planner_input_trajectory=rp_inputs,
    visualize_scenario=False,
    visualize_control=False,
    save_imgs=False,
    img_saving_path=None
)
```
If you want to have no noise and disturbances, you can use `mpc_for_reactive_planner_no_uncertainty(...)` instead
of `mpc_for_reactive_planner(...)` using the same arguments.
