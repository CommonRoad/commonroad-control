# Model Predictive Control

This page describes the **model predictive controller (MPC) controller**.
To accommodate different optimal control problem (OCP) solvers, vehicle models for prediction, and the option to switch between 
nonlinear and real-time iteration schemes, the implementation is split into two components:
an exchangeable optimal control problem (OCP) solver and a wrapper that provides the initial guess to the OCP solver.

______________________________________________________________________

## Optimal Control Problem

We consider the task of tracking a given reference trajectory $x_{\mathrm{ref}}(\cdot), u_{\mathrm{ref}}(\cdot)$, i.e., the cost function to be minimized is

$$
        \min_{x(\cdot), u(\cdot)} \sum_{k=0}^{N} l_k(x(k),u(k)) + V_f(x(N))
$$

with the time-varying stage cost function
$$
        l_k(x(k),u(k)) = (x(k) - x_{\mathrm{ref}}(k))^T Q (x(k) - x_{\mathrm{ref}}(k)) 
                + (u(k) - u_{\mathrm{ref}}(k))^T R (u(k) - u_{\mathrm{ref}}(k)),
$$

and the terminal cost function

$$
        V_f(x(N)) = (x(N) - x_{\mathrm{ref}}(N))^T Q_N (x(N) - x_{\mathrm{ref}}(N)),
$$

with the positive semi-definite weighting matrices $Q$, $R$, and $Q_N$, where $N$ denotes the prediction horizon.
While minimizing the cost function, the following constraints have to be satisfied:

- dynamic feasibility: $\forall k \in \{0,\dots,N-1\}: x(k+1) = f(x(k),u(k),0)$ and $x(0)=x_\mathrm{init}$, where $x_\mathrm{init}$ denots the (measured) initial state
- bounded control inputs: $\forall k \in \{0,\dots,N-1\}: u_{lb} \leq u(k) \leq u_{ub}$
- bounded states: $\forall k \in \{0,\dots,N\}: x_{lb} \leq x(k) \leq x_{ub}$
- (combined) acceleration constraints: $\forall k \in \{0,\dots,N\}: a_{\mathrm{comb}}(x(k),u(k)) \leq 1$, where $a_{\mathrm{comb}}(x(k),u(k))$ returns the normalized combined acceleration, see the [vehicle models documentation](/docs/core_api/vehicle_dynamics/interfaces.md).

______________________________________________________________________

## Initial Guess Generation

Since the vehicle dynamics function is usually nonlinear, the OCP describes above is a nonlinear program and, thus, an initial guess is required for optimization.
The MPC wrapper for the OCP solvers provides the following approaches for computing an initial guess at the $j$th time step:

### User-provided Initial Guess

An initial guess for both the state and input trajectory can be provided by the user.
If an initial guess is provided

### Shift Old Solution

If no initial guess is provided by the user and a solution of the OCP at the previous ($j-1$st) time step is available, the initial guess can be chosen as follows:

- State trajectory: $x^*(1|j-1), \dots, x^*(N|j-1), x_{\mathrm{ref}}(j+N)$
- Input trajectory: $u^*(1|j-1), \dots, u^*(N-1|j-1), u_{\mathrm{ref}}(j+N-1)$

where $\square^*(\cdot|j-1)$ denotes the optimal solution of the OCP at the $j-1$st time step.

### Linear Interpolation

If no initial guess is provided by the user and no old solution is available, we compute an initial guess by means of linear interpolation:

- State trajectory: linear interpolation between the initial state $x_\mathrm{init}$ and the final reference state $x_{\mathrm{ref}}(j+N)$
- Input trajectory: set all control inputs to zero.

______________________________________________________________________

## Interface Implementation

The MPC class implements the `ControllerInterface`.

### Constructor

For initialization of the MPC, an instance of an optimal control solver (see below) is required.

```python
mpc = ModelPredictiveControl(
        ocp_solver
)     
```

### Computation of Control Input

If no initial guess is provided, the (optimal) control input `u0_opt`$=u(0|j)$ given the current state `x0` can be computes as follows:

```python
u0_opt = mpc.compute_control_input(
        x0,
        x_ref,
        u_ref
)
```

where `x_ref` and `u_ref` denote the snippet of the reference trajectory from time step $j$ to time step $N/N-1$.
If an initial guess `x_init` for the state trajectory and `u_init` for the input trajectory are provided, the control input is obtained by running:

```python
u0_opt = mpc.compute_control_input(
        x0,
        x_ref,
        u_ref,
        x_init,
        u_init
)
```

______________________________________________________________________

## OCP Solvers

### OCP Solver Interface

To accomodate different OCP solvers, every OCP solver has to implement the `OptimalControlSolverInterface`.
The following parameters are required to setup an OCP solver:


| Parameter       | Variable type                                | Description                                                                                                                                                                                    |
|-----------------|----------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `vehicle_model` | `VehicleModelInterface`                      | Vehicle model for predicting future states and computing the combined acceleration.                                                                                                            |
| `sidt_factory`  | `StateInputDisturbanceTrajectoryFactoryInterface` | Required for converting the solution of the OCP to instances of state/input/trajectory dataclasses, see the [vehicle models documentation](/docs/core_api/vehicle_dynamics/interfaces.md)<br/> |
| `horizon`       | `int`                                        | Number of (discrete) time steps for prediction.                                                                                                                                                |
| `delta_t`       | `float`                                      | Sampling time in seconds for the OCP.                                                                                                                                                          |
| `ocp_parameters` | `OCPSolverParameters`  | Provides parameters of the OCP solver set by the user, like the value of penalty weights for penalizing theviolation of softened constraitns. |

```python
ocp_solver = OptimalControlSolverInterface(
        vehicle_model,
        sidt_factory,
        horizon,
        delta_t,
        ocp_parameters
)
```

The OCP can be solved by running:

```python
x_opt, u_opt = ocp_solver.solve(
        x0,
        x_ref,
        u_ref,
        x_init,
        u_init,  
)
```

where `x_opt` and `u_opt` are the (predicted) optimal state and input trajectory, respectively.

### Successive Convexification

The `OptimalControlSCvx` solver is based on the successive convexification algorithm proposed in 

[T. P. Reynolds et al. "A Real-Time Algorithm for Non-Convex Powered Descent Guidance", AIAA Scitech Forum, 2020](https://arc.aiaa.org/doi/10.2514/6.2020-0844)

This algorithm solves a sequence of approximations of the nonlinear OCP by convexifying the nonlinear OCP around the solution from the previous convexification step.
The algorithm terminates if the sequence of convex programming solutions has converged or a user-defined number of iterations (see below) is exceeded.

In contrast to a sequential quadratic programming algorithm, nonlinear convex constraints are not linearized but can be solved exactly.
An example is the quadratic combined acceleration constraints (of course, the longitudinal and lateral acceleration migth still be non-convex functions that must be linearized).

We use [CVXPY](https://www.cvxpy.org/) for modeling and [CLARABEL](https://clarabel.org/stable/) for solving the convex approximation of the nonlinear OCP.

#### Solver Parameters

The number of iterations (maximum number of convex programming approximations of the nonlinear OCP) can be set as follows:

```python
ocp_parameters = SCvxParameters(max_iterations=5)
```

A linear MPC or real-time iteration scheme can be implemented by setting the maximum number of iterations to 1.

______________________________________________________________________

## Implementation

The following API documentation is auto-generated from the Python source code.

:::commonroad_control.control.model_predictive_control.model_predictive_control
:::commonroad_control.control.model_predictive_control.optimal_control.optimal_control
:::commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx
