import os
import time
import copy
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# third party
from PIL import Image

# commonroad
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.geometry.shape import Circle
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad.scenario.scenario import Scenario



# typing
from typing import List, Union, Any

from commonroad_control.vehicle_dynamics.kinematic_single_track.kst_trajectory import KSTTrajectory


def visualize_trajectories(
    scenario: Scenario,
    planning_problem: PlanningProblem,
    planner_trajectory: Union[KSTTrajectory, Any],
    controller_trajectory: Union[KSTTrajectory, Any],
    vehicle_width: float = 1.8,
    vehicle_length: float = 4.5,
    size_x: float = 10.0,
    save_img: bool = False,
    save_path: Union[str, Path] = None,
    opacity_planning: float = 0.3,
    opacity_control: float = 1.0,
    use_icon_controlled_traj: bool = True,
    use_icon_planned_traj: bool = False
) -> None:
    for step in planner_trajectory.steps:
        plt.cla()

        # get plot limits from reference idm_path
        plot_limits: List[float] = obtain_plot_limits_from_reference_path(
            planner_trajectory,
            margin=20
        )
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (
            plot_limits[3] - plot_limits[2]
        )

        renderer = MPRenderer(
            plot_limits=plot_limits, figsize=(size_x, size_x / ratio_x_y)
        )
        renderer.draw_params.dynamic_obstacle.draw_icon = True
        renderer.draw_params.dynamic_obstacle.show_label = True
        renderer.draw_params.time_begin = step

        scenario.draw(renderer)

        scenario.lanelet_network.draw(renderer)
        planning_problem.draw(renderer)

        draw_params = copy.copy(renderer.draw_params)
        draw_params.dynamic_obstacle.trajectory.draw_trajectory = False
        draw_params.dynamic_obstacle.show_label = False
        draw_params.planning_problem.initial_state.state.draw_arrow = False
        draw_params.time_begin = step

        # planned
        planner_vehicle = planner_trajectory.to_cr_dynamic_obstacle(
            vehicle_width=vehicle_width,
            vehicle_length=vehicle_length,
            vehicle_id=30000,
        )
        draw_params.dynamic_obstacle.draw_icon = use_icon_planned_traj
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.opacity = opacity_planning
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#000000"
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#808080"
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.zorder = 20
        planner_vehicle.draw(renderer, draw_params=draw_params)

        # controlled
        draw_params = copy.copy(renderer.draw_params)
        controller_vehicle = controller_trajectory.to_cr_dynamic_obstacle(
            vehicle_width=vehicle_width,
            vehicle_length=vehicle_length,
            vehicle_id=30001,
        )
        draw_params.dynamic_obstacle.draw_icon = use_icon_controlled_traj
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.opacity = opacity_control
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.facecolor = "#E37222"
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.edgecolor = "#9C4100"
        draw_params.dynamic_obstacle.vehicle_shape.occupancy.shape.zorder = 50
        controller_vehicle.draw(renderer, draw_params=draw_params)

        # draw scenario and renderer
        renderer.render()
        plt.title(f"Time step = {step}")

        if save_img:
            save_file: str = os.path.join(
                save_path, str(scenario.scenario_id) + "_" + str(step) + ".png"
            )
            os.makedirs(save_path, exist_ok=True)  # Ensure the directory exists
            plt.savefig(save_file, format="png")
        else:
            plt.show()

def obtain_plot_limits_from_reference_path(
    trajectory: Union[KSTTrajectory, Any],
    margin: float = 10.0
) -> List[int]:
    """
    Obtrains plot limits from reference idm_path
    :param reference_path: reference idm_path (2,) np.ndarray
    :return: list [xmin, xmax, ymin, xmax] of plot limits
    """
    arr = np.asarray([[point.position_x, point.position_y] for point in trajectory.points.values()])

    x_min = min(arr[:, 0])
    x_max = max(arr[:, 0])
    y_min = min(arr[:, 1])
    y_max = max(arr[:, 1])

    plot_limits = [x_min - margin, x_max + margin, y_min - margin, y_max + margin]
    return plot_limits


def make_gif(
    path_to_img_dir: Union[Path, str],
    scenario_name: str,
    num_imgs: int,
    duration: float = 0.1,
    abort_img_threshold: int = 100,
) -> None:

    if (
        not os.path.exists(path_to_img_dir)
        or not os.path.isdir(path_to_img_dir)
        or not os.path.isabs(path_to_img_dir)
    ):
        raise FileNotFoundError(
            f"image dir {path_to_img_dir} must exist, be a directory and be absolute"
        )

    # get all files in dir
    imgs = sorted(
        [el for el in os.listdir(path_to_img_dir) if ".png" in el],
        key=lambda x: int(x.split(".")[0].split("_")[-1]),
    )

    print("creating gif")

    # poll until all imgs ware saved
    cnt = 0
    while len(imgs) != num_imgs and cnt < 50:
        imgs = sorted(
            [el for el in os.listdir(path_to_img_dir) if ".png" in el],
            key=lambda x: int(x.split(".")[0].split("_")[-1]),
        )
        time.sleep(0.2)
        cnt += 1

    if cnt == abort_img_threshold:
        raise ValueError("Could not find all expected imgs")

    imgs_pil = [Image.open(os.path.join(path_to_img_dir, img)) for img in imgs]
    output_path = os.path.join(path_to_img_dir, scenario_name + ".gif")

    imgs_pil[0].save(
        output_path,
        save_all=True,
        append_images=imgs_pil[1:],
        duration=duration,
        loop=0,
    )
