# Double Integrator Model

This page describes the **double integrator** vehicle model, which abstracts the vehicle motion via fully decoupled longitudinal and lateral dynamics in a curvilinear coordinate system.

**Currently, we do not support state conversion from the double integrator model to the kinematic/dynamic bicycle model or vice versa!**

See also the [vehicle model interface documentation](/docs/vehicle_dynamics/interfaces.md) for additional information.

---

## Overview

The double integrator model captures:
- 
---

## State Vector

The state vector is defined as:

$$
\mathbf{x} =
\begin{bmatrix}
s \\
d \\
v_s \\
v_d
\end{bmatrix}
$$

where:

| Symbol | Description                                           |
|--------|-------------------------------------------------------|
| $s, d$ | Position in a given curvilinear coordinate system.    |
| $v_s$  | Longitudinal velocity (along the reference path)      |
| $v_d$  | Lateral velocity (with respect to the reference path) |

---

## Control Inputs

The control input vector is:

$$
\mathbf{u} =
\begin{bmatrix}
a_s \\
a_d
\end{bmatrix}
$$

where:

| Symbol | Description |
|--------|------------|
| $a_s$  | Longitudinal acceleration |
| $a_d$  | Lateral acceleration |

Input bounds are derived from the vehicle parameters `a_long_max`, `a_lat_max`.

---

## Continuous-Time Dynamics

The nominal (continuous-time) dynamics are governed by the following set of differential equations:

---
### Longitudinal Motion
$$
\dot{v_s} = u_1
$$

---
### Lateral Motion

$$
\dot{v_d} = u_2
$$

---

## State Bounds

The velocity bounds are enforced as:

$$
v_{x,\min} \le v_x \le v_{x,\max}, \quad
v_{y,\min} \le v_y \le v_{y,\max}
$$

with typical values:

- $v_x \in [0, 10]$ m/s
- $v_y \in [-2, 2]$ m/s

---

## Normalized Accelerations

Normalized longitudinal and lateral accelerations are computed as:

$$
a_{\mathrm{long,norm}} = \frac{a_x}{a_{\mathrm{long,max}}}
$$

$$
a_{\mathrm{lat,norm}} = \frac{a_y}{a_{\mathrm{lat,max}}}
$$

---

## Time Discretization

In contrast to the nonlinear vehicle models, we use the matrix exponential to obtain the nominal discrete-time dynamics of the double integator model.

---

## Implementation

The following API documentation is **auto-generated** from the Python source code.

:::commonroad_control.vehicle_dynamics.double_integrator.double_integrator
:::commonroad_control.vehicle_dynamics.double_integrator.di_state
:::commonroad_control.vehicle_dynamics.double_integrator.di_input
:::commonroad_control.vehicle_dynamics.double_integrator.di_trajectory
:::commonroad_control.vehicle_dynamics.double_integrator.di_disturbance
:::commonroad_control.vehicle_dynamics.double_integrator.di_noise
:::commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory
