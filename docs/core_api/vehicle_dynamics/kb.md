# Kinematic Bicycle Model

This page describes the **kinematic bicycle** model.  
The reference point for the vehicle dynamics is the **center of gravity (CoG)**.

See also the [vehicle model interface documentation](/docs/vehicle_dynamics/interfaces.md) for additional information.

---

## State Vector

The state vector is defined as:

$$
\mathbf{x} =
\begin{bmatrix}
x \\
y \\
v \\
\psi \\
\delta
\end{bmatrix}
$$

where:

| Symbol | Description                        |
|------|------------------------------------|
| $x, y$ | Global position of the vehicle CoG |
| $v$ | (Total) Velocity                   |
| $\psi$ | Heading angle                      |
| $\delta$ | Steering angle                     |

---

## Control Inputs

The control input vector is given by:

$$
\mathbf{u} =
\begin{bmatrix}
a \\
\dot{\delta}
\end{bmatrix}
$$

where:

| Symbol | Description               |
|------|---------------------------|
| $a$ | Longitudinal acceleration |
| $\dot{\delta}$ | Steering angle velocity   |

Input bounds are derived from the vehicle parameters `a_long_max`, `steering_angle_velocity_max`.

---

## Vehicle Parameters

The model uses the following parameters:

| Symbol | Description |
|--------|-------------|
| $l_f$  | Distance from CoG to front axle |
| $l_r$  | Distance from CoG to rear axle |
| $l_{\mathrm{wb}}$ | Wheelbase ($l_f + l_r$) |

---

## Kinematic Relations

The continuous-time dynamics of the kinematic bicycle are:

### Slip Angle

$$
\beta = \arctan\left(\tan(\delta) \frac{l_r}{l_\mathrm{wb}}\right)
$$

### Position Dynamics

$$
\dot{x} = v \cos(\psi + \beta)
$$

$$
\dot{y} = v \sin(\psi + \beta)
$$

### Longitudinal and Heading Dynamics

$$
\dot{v} = u_1
$$

$$
\dot{\psi} = \frac{v \sin(\beta)}{l_r}
$$

### Steering Dynamics

$$
\dot{\delta} = u_2
$$

---

## Normalized Accelerations

The model provides the normalized longitudinal acceleration

$$
a_{\mathrm{long,norm}} = \frac{u_1}{a_{\mathrm{long,max}}}
$$

and the normalized lateral acceleration:
$$
a_{\mathrm{lat}} = v \cdot \dot{\psi}$$
$$
a_{\mathrm{lat,norm}} =
\frac{a_{\mathrm{lat}}}{a_{\mathrm{lat,max}}}
$$


---

## Implementation

The following API documentation is **auto-generated** from the Python source code.

:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_trajectory
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_disturbance
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_noise
:::commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sidt_factory

