# CommonRoad Control

![example.gif](./assets/example.gif)
**CommonRoad Control** is an open-source toolbox for motion control and closed-loop simulation in autonomous driving.
Our toolbox offers various model-based and model-free controllers that are compatible with multiple motion planners
as well as a simulation module with different vehicle dynamics models.

Our toolbox has easy API calls for fast integration in control and motion planning projects
and our overall architecture allows for the modular design of custom motion planning and control pairs.

## :hammer_and_wrench: Installation

![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)
![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)  
We support Ubuntu>=20.4 and Python>=3.10.

```bash
pip install commonroad-control
```

If you want to also install supported motion planners, install them manually using, e.g.,

```bash
pip install commonroad-reactive planner
```

or clone CommonRoad-control from source and:

```bash
poetry install --with planner
```

## :book: Documentation and examples

The [CommonRoad Control Documentation](https://optimal-control-ad-personal-repos-bb7de67c51e7f51ec3bda98b62f32.pages.gitlab.lrz.de/) offers examples and API documentation.
For easy integration in your project, we recommend using either the [CommonRoad easy API](https://optimal-control-ad-personal-repos-bb7de67c51e7f51ec3bda98b62f32.pages.gitlab.lrz.de/examples/minimal_examples/)
or follow the step-by-step examples to use our modular parts for your own controller and planner.

## :computer: Source code

Our [CommonRoad Control github page](https://github.com/CommonRoad/commonroad-control) contains a mirror of our gitlab source code.

## Contributors

Lukas Schäfer: lukas.schaefer[at]tum.de  
Tobias Mascetta: tobias.mascetta[at]tum.de  
Sven Pflaumbaum: sven.pflaumbaum[at]tum.de  
Gerald Würsching: gerald.wuersching[at]tum.de  
