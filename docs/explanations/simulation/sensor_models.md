# Sensor Models

This page describes the **sensor models** used for simulating uncertain measurements.
To enable seamless interchangeability of different sensor models, a common interface including a dataclass
for sensor outputs are provided.
The implemented concrete models are (briefly) presented at the end of this file.

**Currently, we only support full state feedback.**

______________________________________________________________________

## Dataclass Objects

Measured outputs are stored as instances of the corresponding `OutputInterface` dataclass.
For full state feedback, the measured state can also be stored as an instanceof the corresponding
[`StateInterface`](/docs/core_api/vehicle_dynamics/interfaces.md) dataclass.

The output objects can be converted to `numpy` arrays using their `convert_to_array()` method.

______________________________________________________________________

## Overview and Interface Concept

All sensor models implement the same abstract interface, `SensorModelInterface`.

Every vehicle model must implement the output function:
$$
y= h(x,u,\\nu) = h\_{\\mathrm{nominal}}(x,u) + {\\nu}
$$
where the nominal output function $h\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u})$ computes the nominal measurement
from the curretn state $x$ and control input $u$.
The (random) sensor noise is denoted by ${\\nu} \\in \\mathbb{R}^{\\nu}$.

The nominal output function $h\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u})$ must be implemented via the method:

```python
def _output_function_cas(x, u)
```

For public access, the following method is implemented:

```python
def nominal_output(x, u,)
```

This function serves as a wrapper for $h\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u})$ and accepts states and
control inputs that are represented as instances of the respective dataclass.

The noisy output function $h(x,u,\\nu)$ can be evaluated via the method

```python
def measure(x, u, rand_noise)
```

where the default value of the optional input argument `rand_noise` is `True`. If `rand_noise=False`, the output
of `measure` is the output of the nominal output function $h\_{\\mathrm{nominal}}(\\mathbf{x}, \\mathbf{u})$.

### Constructor

```python
SensorModelInterface(
        noise_model,
        state_output_factory,
        dim,
        state_dimension,
        input_dimension,
    )
```

with the parameters:

| Parameters | Variable type | Description |
|-----------|-----------------------------------------------------------------------------------------------------|-------------------------------------------------------------------|
| noise_model | [`UncertaintyModelInterface`](/docs/core_api/simulation/uncertainty_models.md) | Uncertainty model for generating (random) sensor noise. |
| state_output_factory | [`StateInputDisturbanceTrajectoryFactoryInterface`](/docs/core_api/vehicle_dynamics/interfaces.md) | Create output/state from numpy array (output of output function). |
| dim | `int` | Dimension of the output vector. |
| state_dimension | `int` | Dimension of the state vector. |
| input_dimension | `int` | Dimension of the control input vector. |

______________________________________________________________________

## Implemented Sensor Models

______________________________________________________________________

### Full State Feedback

Nominal output function:
$$
h\_{\\mathrm{nominal}}(x,u) = x
$$

Please note that each vehicle model requires a model-specific output dataclass, which are located alongside the
respective vehicle model implementation.

______________________________________________________________________
