# Concept

![example_zam_over.gif](../assets/example_zam_over.gif)

**CommonRoad-Control** closes the gap between motion planning and control research for autonomous driving. 
Many planning benchmarks and open-source projects do not consider the intertwined nature of planning and control. 
Only when analyzed with a closed-loop state-of-the-art controller and a dynamics simulation can motion planning algorithms be truely evaluated.  

Another major gap we aim to close with that project, is the lack of open-source control toolboxes that are fully compatible
with planning benchmarks yet modular enough to easily integrate novel controllers or motion planners.  

Finally, we provide a dedicated numerical simulation using ODE-solvers and kinematic or dynamic models of different fidelity to 
simulate the application of the controller outputs to the plant, including noise and disturbances.


## Easy use with the Easy API
The [Easy API](../examples/minimal_examples.md) of our toolbox allows for the seemless plug-and-play of our implemented controllers. 
It is targeted at motion planning researchers that already use CommonRoad and its planners and simply
want to test their planning results with an out-of-the-box controller and simulation.


## Modularity
The control loop and the interdependence of its constituting parts make it notoriously difficult to implement 
in a modular fashion. Our toolbox offers modules for all major parts of the control loop, such as:  

- model-free and model-based controllers;
- the dynamics simulation;
- kinematic and dynamic models;
- uncertainties (noise and disturbances);
- sensor models (state feedback); and
- the planner integration.

Through the use of interfaces (= base classes), we give users the ability to integrate their own modules fairly easily.


### Modularity of kinematic and dynamic models
Our architecture is specifically made to easily switch between different kinematic and dynamic models for planning, control and simulation.  
The constituting parts of each model are:  

- the state; 
- the input; 
- the trajectory of state of input; 
- the disturbances;
- the noise; and
- the factory that creates states, inputs, disturbances and trajectories (sidt-factory) for the model.  

Each constituting part has an interface (base class), which enforces an implementation compliant with the rest of the toolbox.
All constituting parts inherite from their respective interface. Respecting these interfaces allows users to easily incorporate their own work.



## Integration of your own work
We offer detailes tutorials on how the integrate your own motion planner, controller or vehicle model in these [tutorials](../integrate_your_own_work/planner_integration.md).
You can also follow our [long examples](../examples/long_examples.md) and replace our modules with your own work.





