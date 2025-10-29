from pathlib import Path
import os
import warnings
import matplotlib.pyplot as plt

from commonroad_control.vehicle_dynamics.trajectory_interface import TrajectoryInterface

from typing import Optional, List, Union

def visualize_reference_vs_actual_states(
    reference_trajectory: Union[TrajectoryInterface],
    actual_trajectory: Union[TrajectoryInterface],
    time_steps: List[int] = None,
    state_names: Optional[List[str]] = None,
    save_img: bool = False,
    save_path: Union[str, Path] = None) -> None:
    """
    Plots selected components of the reference and actual (simulated or driven) trajectories as well as the respective tracking error.
    :param reference_trajectory: reference trajectory
    :param actual_trajectory: actual (simulated or driven) trajectory
    :param time_steps: simulation time steps
    :param state_names: (optional) list of state components, which should be plotted - valid values are attribute names of the StateInterface objects
    :param save_img: boolean indicting whether the plot should be saved or not
    :param save_path: path for saving the plot
    :return:
    """

    print(f"visualizing control")

    if state_names is None:
        state_names = vars(reference_trajectory.initial_point).keys()

    # check which items of state_names are defined for the reference trajectory and the actual trajectory
    present_ref_state_names = [a for a in state_names if hasattr(reference_trajectory.initial_point, a)]
    present_actual_state_names = [a for a in state_names if hasattr(actual_trajectory.initial_point, a)]
    plot_state_names = list(set(present_ref_state_names) & set(present_actual_state_names))
    if not plot_state_names:
        warnings.warn(f"No matching state names for the reference and actual trajectory "
                      f"- nothing to be plotted.")
        return
    elif len(plot_state_names) < len(state_names):
        removed_names = list(set(state_names) - set(plot_state_names))
        warnings.warn(f"The following state names are not defined for the reference or actual trajectory "
                      f"{removed_names} and will not be plotted.")

    # plot reference and actual trajectories
    fig, axes = plt.subplots(
        nrows=len(plot_state_names),
        ncols=1,
        figsize=(16, 12)
    )
    plt.title(f"Desired vs. Actual state")

    for ii in range(len(plot_state_names)):
        reference_state_val: List[float] = \
            [getattr(reference_trajectory.get_point_at_time_step(kk),plot_state_names[ii]) for kk in time_steps]
        actual_state_val: List[float] = \
            [getattr(actual_trajectory.get_point_at_time_step(kk), plot_state_names[ii]) for kk in time_steps]

        axes[ii].plot(time_steps, reference_state_val, label=f"reference", color="blue")
        axes[ii].plot(time_steps, actual_state_val, label=f"actual", color="orange")
        axes[ii].title.set_text(plot_state_names[ii])
        axes[ii].legend()

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

    # plot deviation
    fig_err, axes_err = plt.subplots(
        nrows=len(plot_state_names),
        ncols=1,
        figsize=(16, 12)
    )
    plt.title(f"Error")

    for ii in range(len(plot_state_names)):
        reference_state_val: List[float] = \
            [getattr(reference_trajectory.get_point_at_time_step(kk),plot_state_names[ii]) for kk in time_steps]
        actual_state_val: List[float] = \
            [getattr(actual_trajectory.get_point_at_time_step(kk), plot_state_names[ii]) for kk in time_steps]
        error: List[float] = [actual_state_val[kk] - reference_state_val[kk] for kk in range(len(reference_state_val))]

        axes_err[ii].plot(time_steps, error, label=f"error", color="red")
        axes[ii].title.set_text(plot_state_names[ii])
        axes_err[ii].legend()

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

    return
