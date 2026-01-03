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
- kinematic and dynamic vehicle dynamics models;
- uncertainties (sensor noise and disturbances);
- sensor models (e.g., full state feedback); and
- the planner integration.

Through the use of interfaces (= base classes), we give users the ability to integrate their own modules fairly easily.

## Integration of your own work

We offer detailes tutorials on how the integrate your own [motion planner](../integrate_your_own_work/planner_integration.md), [controller](../integrate_your_own_work/controller_integration.md) or [vehicle model](../integrate_your_own_work/dynamics_integration.md) in these tutorials.
You can also follow our long examples for the [PID controller](../examples/long_example_pid.md) and the [MPC](../examples/long_example_mpc.md) and replace our modules with your own work.
