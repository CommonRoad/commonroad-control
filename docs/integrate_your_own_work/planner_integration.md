# Integrate your own planner
This tutorial gives an overview on how to integrate your own motion planner.

## Overview
1. Determine the underlying vehicle parameters of your planner.
2. Choose which kinematic or dynamic model should represent your planner.
3. Choose which kinematic or dynamic model the simulation should use.
4. (Optional) If you are using model-based controllers, choose which kinematic or dynamic model the controller should use.
5. (Optional) Choose a sensor and uncertainty model.
6. Write the planner converter for your planner.  
7. Integrate your planner converter.


## Example
Let us integrate the `CommonRoad-Reactive-Planner` as an example for you to follow for your own planner.
You can follow the [long examples](../examples/long_examples.md) as we go through the tutorial.  
We assume that your planner is able to solve CommonRoad scenarios.

### 1. Determine the underlying vehicle parameters
You can either use out-of-the-box vehicle parameters from a BMW 3 series or inherite from the `vehicle_parameters.py` interface.
For a detailed documentation on the interfaces and provided parameters, confer the corresponding section in the [Core API documentation](../core_api/vehicle_parameters/interfaces.md).
In our case, we choose the existing `BMW3series`.  

Note that if you want to use gaussian noise and disturbances, the mean and std. deviation should also be given by the vehicle parameters.


### 2. Choose the kinematic or dynamic model of your planner
Our toolbox uses the kinematic or dynamic (k/d) model of the planner as the input for the controller. Another -- potentially
different -- k/d model is used to simulate the dynamics of the system with applied controller input, noise and disturbances.
If the controller is model-based, e.g. an MPC, it uses its own k/d model inside.  

The output of your planner must be converted into a supported k/d model or
write your own k/d model by inheriting from the respective interfaces. **Take special care to determine whether your planner
plans from the center of gravity (COG) or the rear-axle (RA).**

In our case, the `CommonRoad-Reactive-Planner` already outputs state and input trajectories that are close 
to the `kinematic bicycle model` (KB) of our toolbox, so we simply choose KB as the planner model. 


### 3. Choose which kinematic or dynamic model the simulation should use
It makes sense to choose a higher-fidelity k/d model for the simulation so that you can check the performance 
of your controller and planner against a more realistic (but more computationally expensive) simulation.  

In our case, we choose the `dynamic bicycle model` (DB) of our toolbox, as it incorporates dynamic information
in contrast to the KB we used for the planner


### 4. (Optional) Choose which kinematic or dynamic model to use for model-based controllers
If you want to use model-based controllers like MPC, choose a k/d model that is used by them. 
It makes sense to use a lower-fidelity model than in simulation but an equal or better one that your planner uses.  

In our case, we choose to use KD again, as this is a reasonably fast model for control.


### 5. (Optional) Choose a sensor and uncertainty model.
If you want to use uncertainty and sensor models (aka. specific state feedback), you can either choose the once implemented
in our toolbox or write your own by inheriting from the respective interfaces.  

We choose the `Gaussian disturbance and noise` and `full state feedback` from our toolbox. 


### 6. Write your planner converter
This is were the coding begins. You have chosen vehicle parameters, a k/d model each for the planner and the simulation, respectivel, and optionally 
optionally also for a model-based controller. 

**Take special care to determine whether your planner
plans from the center of gravity (COG) or the rear-axle (RA). Make sure that you convert them correctly. Our
models use COG!**

You can use the [reactive planner converter](../core_api/planning_converter/reactive_planner_converter.md) as a step-by-step template. In here, `p2c` stands for planner
to controller and `c2p` for controller to planner conversion. 

#### Steps
1. Inherite from the `PlanningConverterInterface`. You have to implement all abstract classes.
2. Implement the conversion of a planner state or input (if applicable) into a state or input of the k/b planner model you chose. In the 
`reactive planner converter`, this is `trajectory_p2c_kb` and `sample_p2c_kb` for states.
3. Implement the conversion from the planner k/b model to the planners internal representation. 
In the `reactive planner converter`, this is `trajectory_c2p_kb` and `sample_c2p_kb` for states.
4. Repeat step 2 and 3 (optionally) for the k/b model you chose for the simulation (and optionally the controller).
In the `reactive planner converter`, this is `trajectory_p2c_db` and `sample_p2c_db` for planner to controller.


### 7. Integrate your planner converter
Follow the [long example](../examples/long_examples.md) on how to integrate your controller. 
As a first step, we recommend using KB and DB as we have done here and a model-free PID controller.
Use the PID-example script and only swap the `ReactivePlannerConverter` lines with the converter for your own planner:

```Python
...
rpc = ReactivePlannerConverter() # Swap for your converter
x_ref = rpc.trajectory_p2c_kb(
    planner_traj=rp_states,
    mode=TrajectoryMode.State
)
u_ref = rpc.trajectory_p2c_kb(
    planner_traj=rp_inputs,
    mode=TrajectoryMode.Input
)
...
```


If everything went well, you should already be able to use the PID-controller with Gaussian noise and disturbances 
on your new planner, maybe after some controller parameter tuning.

As a next step, we recommend playing with noise and disturbances. You can take a look how that is done in the
[easy API](../easy_api/easy_pid.md). For example, `NoUncertainty` should work fairly easily if your example
already works with Gaussian uncertainty.  


Next, you can try to use our MPC with your planner, following the [long example](../examples/long_examples.md) 
for MPC and swapping the `ReactivePlannerConverter` for your converter. 


Lastly, you can play with different k/d models. Note, that tuning effort may be necessary.




