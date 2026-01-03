# Dynamic Bicycle Model

This page describes the **dynamic bicycle** model with a **linear tyre model** and **longitudinal load transfer**.
The reference point for the vehicle dynamics is the **center of gravity (CoG)**.

See also the [vehicle model interface documentation](interfaces.md) for additional information.

______________________________________________________________________

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

______________________________________________________________________

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

| Symbol | Description |
|------|---------------------------|
| $a$ | Longitudinal acceleration |
| $\dot{\delta}$ | Steering angle velocity |

Input bounds are derived from the vehicle parameters `a_long_max`, `steering_angle_velocity_max`.

______________________________________________________________________

## Vehicle Parameters

The model uses the following parameters:

| Symbol | Description |
|--------------------|-------------|
| $l_f$ | Distance from CoG to front axle |
| $l_r$ | Distance from CoG to rear axle |
| $l\_{\mathrm{wb}}$ | Wheelbase ($l_f + l_r$) |
| $m$ | mass of the vehicle |
| $h\_{\mathrm{cog}}$ | Height of the CoG|
| $C_f$/$C_r$ | front/ rear cornering stiffness coefficient|

______________________________________________________________________

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

______________________________________________________________________

### Normal Forces with Load Transfer

We account for longitudinal load transfer due to the (commanded) longitudinal acceleration:

$$
F\_{z,f} = \frac{m g l_r - m a h\_{\mathrm{cog}}}{l\_{\mathrm{wb}}}
$$

$$
F\_{z,r} = \frac{m g l_f + m a h\_{\mathrm{cog}}}{l\_{\mathrm{wb}}}
$$

______________________________________________________________________

### Lateral Tyre Forces

The lateral tyre forces follow as:

$$
F\_{c,f} = - C_f \alpha_f F\_{z,f}
$$

$$
F\_{c,r} = - C_r \alpha_r F\_{z,r}
$$

______________________________________________________________________

## Nominal Vehicle Dynamics

The nominal (continuous-time) dynamics are governed by the following set of differential equations:

### Kinematic Relations

$$
\dot{x} = v_x \cos(\psi) - v_y \sin(\psi)
$$

$$
\dot{y} = v_x \sin(\psi) + v_y \cos(\psi)
$$

______________________________________________________________________

### Longitudinal and Lateral Dynamics

$$
\dot{v}_x = \dot{\psi} v_y + u_1 - \frac{F_{c,f} \sin(\delta)}{m}
$$

$$
\dot{v}_y = -\dot{\psi} v_x + \frac{F_{c,f} \cos(\delta) + F\_{c,r}}{m}
$$

______________________________________________________________________

### Yaw and Steering Dynamics

$$
\ddot{\psi} =
\frac{l_f F\_{c,f} \cos(\delta) - l_r F\_{c,r}}{I\_{zz}}
$$

$$
\dot{\delta} = u_2
$$

______________________________________________________________________

## Normalized Accelerations

The model provides the normalized longitudinal acceleration

$$
a\_{\mathrm{long}} = u_1 - \frac{F\_{c,f} \sin(\delta)}{m}
$$
$$
a\_{\mathrm{long,norm}} =
\frac{a\_{\mathrm{long}}}{a\_{\mathrm{long,max}}}
$$
and the normalized lateral acceleration:
$$
a\_{\mathrm{lat}} = \frac{F\_{c,f} \cos(\delta) + F\_{c,r}}{m}
$$
$$
a\_{\mathrm{lat,norm}} =
\frac{a\_{\mathrm{lat}}}{a\_{\mathrm{lat,max}}}
$$

______________________________________________________________________
