# Uncertainty Models

This page describes the **uncertainty models** used to generate (random) disturbances or sensor noise for closed-loop simulation.
To enable seamless interchangeability of different uncertainty models (e.g., Gaussian, uniform distributions), a common
interface including a dataclass for generated uncertainty vectors are provided.
The implemented concrete models are (briefly) presented at the end of this file.

______________________________________________________________________

## Dataclass Objects

Generated uncertainty vectors are stored as instances of the corresponding `UncertaintyInterface` dataclass.
Each [vehicle model](../../core_api/vehicle_dynamics/interfaces.md) must implement its concrete disturbance dataclass.

The uncertainty dataclass objects can be converted to `numpy` arrays using their `convert_to_array()` method.

______________________________________________________________________

## Overview and Interface Concept

All uncertainty models implement the same abstract interface, `UncertaintyModelInterface`.

Let the uncertainty be represented by a random vector $z \in \mathbb{R}^d$ with fixed dimension $d$.
Each uncertainty model provides:

- a **nominal value**
  $$
  \bar{z} = \mathbb{E}[z] \quad \text{(or a user-defined value)}
  $$

- a **sampling operator**
  $$
  w \sim \mathcal{Z}
  $$
  where $\mathcal{Z}$ denotes the underlying distribution.

______________________________________________________________________

### Interface Definition

#### Required Parameters

| Parameter | Variable type | Description |
|-----------------|---------------------------------------------------|---------------------------------------------------------------------------------|
| `dim` | `int` | Dimension of the uncertainty vector |
| `nominal_value` | `np.ndarray`/`List[float]`/`UncertaintyInterface` | User-defined nominal value or default value defined in the concrete child class |

#### Required Methods

| Method | Description |
|------|-------------------------------------------------------------------------------|
| `nominal_value()` | Returns the nominal value |
| `sample_uncertainty()` | Draws a random sample from the underlying uncertainty distribution |

______________________________________________________________________

## Implemented Uncertainty Models

______________________________________________________________________

### No Uncertainty

Dummy uncertainty model if, for instance, no disturbances or sensor noise is desired.

Default nominal value: $\bar{z} = 0$

#### Additional parameters:

No additional parameters required.

______________________________________________________________________

### Gaussian Distribution

Models a normally distributed random variable, i.e., $z \sim \mathcal{N}(\mu,\sigma^2)$ with mean $\mu \in \mathbb{R}^d$ and standard deviation $\sigma \in \mathbb{R}^d$.

Default value: $\bar{z} = \mu$

#### Additional parameters:

| Parameter | Variable type and default value | Description |
|-----------------|----------------------------------------------------------|--------------------|
| `mean` | `np.ndarray`/`List[float]`/`UncertaintyInterface` | Mean |
| `std_deviation` | `np.ndarray`/`List[float]`/`UncertaintyInterface` | Standard deviation |

______________________________________________________________________

### Uniform Distribution

Models a uniformly distributed random variable $z$ whose value lies between certain bounds $a \in \mathbb{R}^d$, $b \in \mathbb{R}^d$ with $a \leq b$.

Default nominal value: $\bar{z} = \frac{1}{2}(a + b)$

#### Additional parameters:

| Parameter | Variable type and default value | Description |
|---------------|----------------------------------------------------------|-------------|
| `lower_bound` | `np.ndarray`/`List[float]`/`UncertaintyInterface` | Lower bound |
| `upper_bound` | `np.ndarray`/`List[float]`/`UncertaintyInterface` | Upper bound |
