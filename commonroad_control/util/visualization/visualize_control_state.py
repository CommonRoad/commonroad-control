from pathlib import Path
import os

import matplotlib.pyplot as plt

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory import KBTrajectory
from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface

from typing import Optional, List, Union

def visualize_desired_vs_actual_states(
    desired_states: Union[TrajectoryInterface, KBTrajectory],
    actual_states: Union[TrajectoryInterface, KBTrajectory],
    time_steps: List[int],
    state_dim: int,
    state_names: Optional[List[str]] = None,
    save_img: bool = False,
    save_path: Union[str, Path] = None,
) -> None:

    print(f"visualizing control")

    if state_names is None:
        state_names = ["" for _ in range(state_dim)]

    # TODO: sanity check input


    fig, axes = plt.subplots(
        nrows=state_dim,
        ncols=1,
        figsize=(16, 12)
    )
    plt.title(f"Desired vs. Actual state")

    for i in range(state_dim):
        desired_state_val: List[float] = [desired_states.points[step].convert_to_array()[i] for step in time_steps]
        actual_state_val: List[float] = [actual_states.points[step].convert_to_array()[i] for step in time_steps]

        axes[i].plot(time_steps, desired_state_val, label=f"desired state x_{i}", color="blue")
        axes[i].plot(time_steps, actual_state_val, label=f"actual state x_{i}", color="orange")

        axes[i].legend()

    plt.tight_layout()  # Avoid overlap


    if save_img and save_path is not None:
        save_dir = os.path.join(save_path, "control")
        save_file: str = os.path.join(
            save_path, "control", "states.png"
        )
        os.makedirs(save_dir, exist_ok=True)  # Ensure the directory exists
        plt.savefig(save_file, format="png")
    else:
        plt.show()



    fig_err, axes_err = plt.subplots(
        nrows=state_dim,
        ncols=1,
        figsize=(16, 12)
    )
    plt.title(f"Error")

    for i in range(state_dim):
        desired_state_val: List[float] = [desired_states.points[step].convert_to_array()[i] for step in time_steps]
        actual_state_val: List[float] = [actual_states.points[step].convert_to_array()[i] for step in time_steps]
        error: List[float] = [desired_state_val[i] - actual_state_val[i] for i in range(len(desired_state_val))]

        axes_err[i].plot(time_steps, error, label=f"error x_{i}", color="red")

        axes_err[i].legend()

    plt.tight_layout()  # Avoid overlap


    if save_img and save_path is not None:
        save_dir = os.path.join(save_path, "control")
        save_file: str = os.path.join(
            save_dir, "error.png"
        )
        os.makedirs(save_dir, exist_ok=True)  # Ensure the directory exists
        plt.savefig(save_file, format="png")
    else:
        plt.show()






