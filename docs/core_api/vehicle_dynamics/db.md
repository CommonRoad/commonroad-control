# Dynamic Bicycle Model

This page describes the **dynamic bicycle** model with a **linear tyre model** and **longitudinal load transfer**.  
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
v_x \\
v_y \\
\psi \\
\dot{\psi} \\
\delta
\end{bmatrix}
$$

where:

| Symbol | Description |
|------|------------|
| $x, y$ | Global position of the vehicle CoG |
| $v_x$ | Longitudinal velocity in body frame |
| $v_y$ | Lateral velocity in body frame |
| $\psi$ | Heading angle |
| $\dot{\psi}$ | Yaw rate |
| $\delta$ | Steering angle |

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

| Symbol             | Description |
|--------------------|-------------|
| $l_f$              | Distance from CoG to front axle |
| $l_r$              | Distance from CoG to rear axle |
| $l_{\mathrm{wb}}$  | Wheelbase ($l_f + l_r$) |
| $m$                | mass of the vehicle |
| $h_{\mathrm{cog}}$ | Height of the CoG|
| $C_f$/$C_r$        | front/ rear cornering stiffness coefficient|



---

## Tyre Model

A **linear tyre model** is used for both front and rear axles.

### Slip Angles

The slip angles are defined as:

$$
\alpha_f = \arctan\left(\frac{v_y + l_f \dot{\psi}}{v_x}\right) - \delta
$$

$$
\alpha_r = \arctan\left(\frac{v_y - l_r \dot{\psi}}{v_x}\right)
$$

---

### Normal Forces with Load Transfer

We account for longitudinal load transfer due to the (commanded) longitudinal acceleration:

$$
F_{z,f} = \frac{m g l_r - m a h_{\mathrm{cog}}}{l_{\mathrm{wb}}}
$$

$$
F_{z,r} = \frac{m g l_f + m a h_{\mathrm{cog}}}{l_{\mathrm{wb}}}
$$

---

### Lateral Tyre Forces

The lateral tyre forces follow as:

$$
F_{c,f} = - C_f \alpha_f F_{z,f}
$$

$$
F_{c,r} = - C_r \alpha_r F_{z,r}
$$

---

## Nominal Vehicle Dynamics

The nominal (continuous-time) dynamics are governed by the following set of differential equations:

### Kinematic Relations

$$
\dot{x} = v_x \cos(\psi) - v_y \sin(\psi)
$$

$$
\dot{y} = v_x \sin(\psi) + v_y \cos(\psi)
$$

---

### Longitudinal and Lateral Dynamics

$$
\dot{v}_x = \dot{\psi} v_y + u_1 - \frac{F_{c,f} \sin(\delta)}{m}
$$

$$
\dot{v}_y = -\dot{\psi} v_x + \frac{F_{c,f} \cos(\delta) + F_{c,r}}{m}
$$

---

### Yaw and Steering Dynamics

$$
\ddot{\psi} =
\frac{l_f F_{c,f} \cos(\delta) - l_r F_{c,r}}{I_{zz}}
$$

$$
\dot{\delta} = u_2
$$

---

## Normalized Accelerations

The model provides the normalized longitudinal acceleration

$$
a_{\mathrm{long}} = u_1 - \frac{F_{c,f} \sin(\delta)}{m}
$$
$$
a_{\mathrm{long,norm}} =
\frac{a_{\mathrm{long}}}{a_{\mathrm{long,max}}}
$$
and the normalized lateral acceleration:
$$
a_{\mathrm{lat}} = \frac{F_{c,f} \cos(\delta) + F_{c,r}}{m}
$$
$$
a_{\mathrm{lat,norm}} =
\frac{a_{\mathrm{lat}}}{a_{\mathrm{lat,max}}}
$$

---

## Implementation

The following API documentation is **auto-generated** from the Python source code.


:::commonroad_control.vehicle_dynamics.dynamic_bicycle.dynamic_bicycle
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_state
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_input
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_trajectory
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_disturbance
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_noise
:::commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory
