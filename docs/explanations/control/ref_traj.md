# Reference Trajectory Factory

This page documents the **reference trajectory factory**, which acts as an interface between the motion planner and the controller.
More precisely, it extracts the required snippet for control from the reference trajectory
(provided by the motion planner).
For most controller, this snippet is a single state or output at a given point in time; for model predictive control (MPC) 
the duration of this snippet corresponds to the prediction horizon.

## Constructor

```python3
ref_traj = ReferenceTrajectoryFactory(
        delta_t_controller,
        sidt_factory,
        mpc_horizon,
        t_look_ahead
)
```

with the parameters

| Parameter            | Variable type and default value                                                                     | Description                                                                                                                                                                                                                                                              |
|----------------------|-----------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `delta_t_controller` | `float`                                                                                             | Sampling time of the controller.                                                                                                                                                                                                                                         |
| `sidt_factory`       | [`StateInputDisturbanceTrajectoryFactoryInterface`](../../core_api/vehicle_dynamics/interfaces.md) | Required for sampling the reference trajectory in between samples of the motion planner. The `sidt_factory` is required for converting states/control inputs to the correct dataclass, see the [vehicle model documentation](../vehicle_dynamics/interfaces.md). |
| `mpc_horizon`        | `int = 0`                                                                                           | (Discrete) Prediction horizon of an MPC. (Only required for MPC).                                                                                                                                                                                                        |
| `t_look_ahead`       | `float=0.0`                                                                                         | Look-ahead time.                                                                                                                                                                                                                                                         |

## Setting the Reference Trajectory

The state and input reference trajectory `state_ref` and `input_ref`, respectively, starting at time `t_0` can be set or updated as follows:

```python3
ref_traj.set_reference_trajectory(
        state_ref, 
        input_ref, 
        t_0
)
```

Note that only one factory instance is required to handle both the state and input reference trajectories.

## Querying the Reference Trajectory

To extract the required snippet from the reference trajectory, only the current (continuous) time `t_now` $\geq$ `t_0` has to be provided. 

```python3
    x_ref, u_ref = ref_traj.get_reference_trajectory_at_time(
        t_now
    )
```
Since the prediction horizon of the MPC or potential look-ahead times are provided upon initialization, the correct duration and initial time of the snippet are determined internally.

- For `delta_t_controller = 0.1` and `mpc_horizon = 20`, `x_ref` and `u_ref` are the snippet of `state_ref` and `input_ref` from `t_now` $\leq t \leq$ `t_now` $+ 2\text{s}$ (sampled every `delta_t_controller` seconds).
- For a PID controller with look-ahead `t_look_ahead` $=0.5\text{s}$, `x_ref` and `u_ref` are the point of `state_ref` and `input_ref` at $t=$`t_now`$+ 0.5\text{s}$. If the reference at $t=$`t_now` is required, the factory can be forced to ignore the lookahead by adding the (optional) input argument `consider_look_ahead=False`.

__