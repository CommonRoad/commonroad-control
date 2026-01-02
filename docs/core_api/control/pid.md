# PID Controller

This page documents the **Proportional–integral–derivative (PID) controller**, which can be used for both longitudinal and lateral control.
Based on the PID controller, a controller for decoupled longitudinal and lateral motion control (`PIDLongLat`) is presented afterwards.
______________________________________________________________________

## Control Law

In continuous-time, the control input at time $t$ is obtained by evaluating 

$$
    u(t)= K_p e(t)  + K_i \int_{0}^{t} e(\tau) d\tau + K_d \dot{e}(t),
$$

with the error $e(t) = y(t) - y_{\mathrm{ref}}(t)$ between the measured output $y(t)$ and the reference output 
$y_{\mathrm{ref}}(t)$ and the gains $K_{\square}$. 
To obtain the discrete-time version of the control law, we employ the following simplifications

$$
   \int_{0}^{t} e(\tau) d\tau \approx \Delta t \sum_{j=0}^{k} e(j) \qquad t=k{\Delta t},
$$
$$
    \dot{e}(t) = \frac{e(k) - e(k-1)}{\Delta t},
$$

and, thus, obtain
$$
    u(k)= K_p e(k)  + K_i \Delta t \sum_{j=0}^{k} e(j) d\tau + K_d \frac{e(k) - e(k-1)}{\Delta t}.
$$

______________________________________________________________________

## Interface Implementation

The `PIDControl` class implements the `ControllerInterface`.

### Constructor

```python
pid = PIDControl(
    kp, 
    ki, 
    kd, 
    delta_t
)
```
with the parameters:

| Parameter | Variable type | Description              |
|-----------|---------------|--------------------------|
| `kp`      | `float`       | Proportional gain        |
| `ki`      | `float`         | Integral gain            |
| `kd`      | `float`         | Derivative gain          |
| `delta_t` | `float`         | Sampling time in seconds |

### Computation of Control Input

Given the measured output `measured_output` and the reference output `reference_output`, the control input is obtained by running:

```python
u = pic.compute_control_input(
        measured_output,
        reference_output,
)
```

______________________________________________________________________

# Decoupled Longitudinal and Lateral PID-Control

The `PIDLongLat` controller combines two PID controllers for longitudinal and lateral control. 
The following outputs for tracking and control inputs are chosen:

|            | Controlled variable                      | Control input             |
|----------------------|------------------------------------------|---------------------------|
| Longitudinal control | Velocity                                 | Longitudinal acceleration |
| Lateral control    | Lateral offset from reference trajectory | Steering angle velocity   |

______________________________________________________________________

## Constructor

```python
PIDLongLat(        
    kp_long,
    ki_long,
    kd_long,
    kp_lat,
    ki_lat,
    kd_lat,
    
)
```

### Required Parameters

| Parameter | Variable type | Description              |
|-----------|---------------|--------------------------|
| `kp`      | `float`       | Proportional gain        |
| `ki`      | `float`         | Integral gain            |
| `kd`      | `float`         | Derivative gain          |
| `delta_t` | `float`         | Sampling time in seconds |


______________________________________________________________________

## Implementation

The following API documentation is auto-generated from the Python source code.

:::commonroad_control.control.pid.pid_control
:::commonroad_control.control.pid.pid_long_lat
