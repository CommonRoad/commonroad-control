# CommonRoad Control
![example.gif](./assets/example.gif)
**CommonRoad Control** is an open-source toolbox for motion control and dynamic simulation in autonomous driving.
Our toolbox offers various model-based and model-free controllers that are compatible with multiple motion planners
as well as a dynamic simulation with different vehicle dynamics models. 

Our toolbox has easy API calls for fast integration in control and motion planning projects
and our overall architecture allows for the modular design of custom motion planning and control pairs.


## :hammer_and_wrench: Installation

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
The [CommonRoad Control Documentation]() offers examples and API documentation.
For easy integration in your project, we recommend using either the [CommonRoad easy API]()
or follow the step-by-step examples to use our modular parts for your own controller and planner.


## :computer: Source code
Our [CommonRoad control github page]() contains a mirror of our gitlab source code.



## Contributers
Lukas Schäfer: lukas.schaefer[at]tum.de  
Tobias Mascetta: tobias.mascetta[at]tum.de  
Sven Pflaumbaum: sven.pflaumbaum[at]tum.de  
Gerald Würsching: gerald.wuersching[at]tum.de  