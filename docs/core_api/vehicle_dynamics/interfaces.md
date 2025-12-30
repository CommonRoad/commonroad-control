# Vehicle Model Interface

This page describes the **VehicleModelInterface**, which defines a **common abstraction layer**
for all vehicle dynamics models used in CommonRoad-Control.
The interface enables seamless interchangeability of different vehicle dynamics models (e.g., kinematic bicycle,
dynamic bicycle) for simulation and control.

______________________________________________________________________

## Dataclass Objects

In addition to the vehicle model itself, we provide dataclass objects for storing vehicle-specific state vectors, inputs, disturbance, and noise vectors as well as trajectories.
The `StateInputDisturbanceTrajectoryFactoryInterface` (short `sidt_factory`) implements the functionality for instantiating these dataclass objects from `numpy` arrays or by providing the respective attributes as individual arguments.

The state, input, disturbance, and noise dataclass objects can be converted to `numpy` arrays using their `convert_to_array()` method for, e.g., numerical simulation.

### State Vector

Each model defines a state vector:

$$
\\mathbf{x} \\in \\mathbb{R}^{n_x}
$$

______________________________________________________________________

### Control Input Vector

Each model defines a control input vector:

$$
\\mathbf{u} \\in \\mathbb{R}^{n_u}
$$

______________________________________________________________________

### Disturbance Vector

Each model defines an additive disturbance to account for, e.g., unmodeled dynamics or external forces:

$$
\\mathbf{w} \\in \\mathbb{R}^{n_w}
$$

where $n_w = n_x$.
Per default, each component of $\\mathbf{w}$ is set to zero unless specified otherwise during instantiation.

______________________________________________________________________

### Noise Vector

For full-state feedback (see also the [sensor model documentation](/docs/simulation/sensor_models.md)), each model defines a full-state noise vector:

$$
\\mathbf{\\nu} \\in \\mathbb{R}^{n\_{\\nu}}
$$

where $n\_{\\nu} = n_x$.
Per default, each component of $\\mathbf{\\nu}$ is set to zero unless specified otherwise during instantiation.

______________________________________________________________________

### Trajectory

A trajectory stores a list of points that are sampled at a constant rate of $1/{\\Delta t}$ starting at initial time $t_0$ and ending at final time $t\_{\\mathrm{final}}$.

Querying of the trajectory is possible at discrete points in time with $t = t_0 + k \\Delta t$ using

```python
def get_point_at_time_step(k)
```

as well as continuous points in time $t$ with $t_0 \\leq t \\leq t\_{\\mathrm{final}}$ using

```python
def get_point_at_time_step(t, sidt_factory)
```

where the point at time $t$ is approximated via linear interpolation between the points at adjacent discrete time steps.
To return the point as an instance of the corresponding dataclass, the vehicle-specific `sidt_factory` is required as an input argument.

Same as the states, inputs, disturbances, and noises, the trajectory supports conversion to a `numpy` array using its `convert_to_array` method.

______________________________________________________________________

## Continuous-Time Dynamics

Every vehicle model must implement the continuous-time dynamics:
$$
\\mathbf{\\dot{x}} = f(\\mathbf{x}, \\mathbf{u}, \\mathbf{w}) = f\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u}) + \\mathbf{w}
$$
where $f\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u})$ denotes the nominal vehicle dynamics.

The continuous-time dynamics must be implemented via the method:

```python
def _dynamics_cas(x, u, w)
```

For public access of the dynamics model (e.g., for simulation), the following method is implemented:

```python
def dynamics_ct(x, u, w)
```

This function serves as a wrapper for the continuous-time dynamics and accepts states, control inputs, and disturbances that are represented as instances of the respective dataclass.

______________________________________________________________________

## Time Discretization

For model predictive control, the interface automatically constructs a **time-discretized nominal model** (i.e., $\\mathbf{w} = \\mathbf{0}$):

$$
\\mathbf{x}\_{k+1} = f_d(\\mathbf{x}\_k, \\mathbf{u}\_k, \\mathbf{0})
$$

using a fixed sampling time $\\Delta t$ provided to the constructor of the class and the classical Runge-Kutta method (aka RK4).

______________________________________________________________________

### Linearization

For linearization/convexification-based optimal control, the interface provides **automatic linearization** of the time-discretized dynamics:
$$
\\mathbf{x}\_{k+1} \\approx f_d(\\bar{\\mathbf{x}}, \\bar{\\mathbf{u}},\\mathbf{0}) + A (\\mathbf{x} - \\bar{\\mathbf{x}}) + B (\\mathbf{u} - \\bar{\\mathbf{u}})
$$
where:

- $A = \\frac{\\partial f_d}{\\partial \\mathbf{x}}$
- $B = \\frac{\\partial f_d}{\\partial \\mathbf{u}}$

These Jacobians are computed symbolically using CasADi.

______________________________________________________________________

## Normalized Accelerations

All models provide the functionality for computing the normalized longitudinal and lateral accelerations.
The vehicle does not exceed the combined acceleration limits if the following constraint is satisfied:
$$
a\_{\\mathrm{long,norm}}^2 + a\_{\\mathrm{lat,norm}}^2 \\leq 1
$$
The corresponding functions are designed to enable the integration of this constraint in an optimal control problem for model predictive control.

______________________________________________________________________

## Input Bounds

Each vehicle model must define (element-wise) input bounds:

$$
\\mathbf{u}_{\\min} \\le \\mathbf{u} \\le \\mathbf{u}_{\\max}
$$

derived from the vehicle parameters.

______________________________________________________________________

______________________________________________________________________

## Implementation

The following API documentation is auto-generated from the Python source code.

:::commonroad_control.vehicle_dynamics.vehicle_model_interface
:::commonroad_control.vehicle_dynamics.state_interface
:::commonroad_control.vehicle_dynamics.input_interface
:::commonroad_control.vehicle_dynamics.trajectory_interface
:::commonroad_control.vehicle_dynamics.disturbance_interface
:::commonroad_control.vehicle_dynamics.full_state_noise_interface
:::commonroad_control.vehicle_dynamics.sidt_factory_interface
