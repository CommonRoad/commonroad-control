# Simulation Module

This page documents the **simulation module**, which provides a generic framework for simulating the closed-loop vehicle models.
For more realistic simulation results, **disturbances** and \[sensor models with **measurement noise** can be included.

The simulation module is **model-agnostic** and supports **any vehicle model implementing the
[`VehicleModelInterface`](/docs/core_api/vehicle_dynamics/interfaces.md)**, such as the [kinematic bicycle model](/docs/core_api/vehicle_dynamics/kb.md)
or the [dynamic bicycle model](/docs/core_api/vehicle_dynamics/db.md) .

______________________________________________________________________

## Problem Formulation

The simulator considers initial value problems of the form
$$
\\dot{x}(t)=f(x(t),u_0,w(t)), \\quad x(0) = x_0
$$

with constant control input $u_0$, a (random) disturbance signal $w(t)$, and a time horizon $t_f \\geq 0$.
The disturbance signal $w(t)$ is assumed to be piece-wise constant and (randomly) sampled at a rate of ${1}/{\\Delta t_w}$ where $0 < \\Delta t_w \\leq t_f$.

## At the end of the time horizon, the output function $h(x,u_0,\\nu)$ of the sensor model is evaluated for the simulated state $x(t_f)$, the constant control input $u_0$, and the sensor noise $\\nu$.

## Architecture

The simulation module requires of the following components:

| Component | Interface | Description |
|-------------------|--------------------------------------------------------------------------------|-------------------------------------------------------|
| Vehicle model | [`VehicleModelInterface`](/docs/core_api/vehicle_dynamics/interfaces.md) | Provides the continuous-time dynamics function. |
| SIDT factory | [`StateInputDisturbanceTrajectoryFactoryInterface`](/docs/core_api/vehicle_dynamics/interfaces.md) | State, input, and trajectory conversion. |
| Disturbance model | [`UncertaintyModelInterface`](/docs/core_api/simulation/uncertainty_models.md) | Uncertainty model for (random) disturbance generation |
| Sensor model | [`SensorModelInterface`](/docs/core_api/simulation/sensor_models.md) | Output function and sensor noise modeling. |

______________________________________________________________________

### Constructor

```python
Simulation(
    vehicle_model,
    sidt_factory,
    disturbance_model,
    sensor_model,
    random_disturbance
    random_noise,
    delta_t_w
)
```

with the additional parameters:

| Parameter | Variable type and default value | Description |
|--------------------|-------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------|
| random_disturbance | `bool = False` | If true, disturbances are sampled randomly, otherwise, the nominal value of the `UncertaintyModelInterface` is applied. |
| random_noise | `bool = False` | If true, sensor noises are sampled randomly, otherwise, the nominal value of the `UncertaintyModelInterface` is applied. |
| delta_t_w | `float = 0.1` | Sampling time for the disturbances in seconds, see below. |

## Simulation

```Python3
sim = Simulation(...)

y, traj_w, traj_nom = sim.simulate(
    x0=initial_state,
    u=constant_control_input,
    t_final=time_horizon
)
```

with the output arguments:

| Output | Interface | Description |
|--------------|-----------------------------------|------------------------------------------------------------------------------------------------------------------------------|
| y | `StateInterface`/`OutputInterface` | (Noisy) measurement - value of the output function |
| traj_w | `TrajectoryInteface` | Perturbed trajectory, stores the simulated states sampled at every $\\Delta t_w$ seconds |
| traj_nom | `TrajectoryInteface` | Nominal trajectory, stores the simulated states of the unperturbed dynamics ($w(t)=0$) sampled at every $\\Delta t_w$ seconds |

By default, the fourth-order Runge-Kutta method with adaptive step size `RK45` is used for integration.
Other methods implemented in `scipy.integrate.solve_ivp` can be selected via the optional input argument `ivp_method`.

______________________________________________________________________

### Nominal and Perturbed Simulation

Two trajectories are propagated in parallel:

- Nominal trajectory:
  Uses the nominal value of the disturbance (usually $w(t)=0$) and represents ideal system behavior.
- Perturbed trajectory:
  Uses randomly sampled disturbances to account for, e.g., external forces or unmodeled dynamics.

If no uncertainty model for the disturbances is available, the `NoUncertainty` model allows purely nominal simulation
(see [`UncertaintyModelInterface`](/docs/core_api/simulation/uncertainty_models.md)).
By default, `NoUncertainty` is employed for disturbance generation, if none is provided to the constructor.

______________________________________________________________________

### Output Functions and Sensor Noise

Please see the [sensor model documentation](/docs/core_api/simulation/sensor_models.md).

By default, `FullStateFeedback` is employed as the sensor model with `NoUncertainty` as the noise model
(see also [`UncertaintyModelInterface`](/docs/core_api/simulation/uncertainty_models.md)), if no
sensor model is provided to the constructor.

______________________________________________________________________

## Implementation

The following API documentation is auto-generated from the Python source code.

:::commonroad_control.simulation.simulation.simulation
