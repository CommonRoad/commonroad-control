# Integrate Your Own Vehicle Dynamics Model

## Overview 
Our architecture is specifically made to easily switch between different kinematic and dynamic models for planning, control and simulation.
Each model requires a set of constituting parts, which have an interface (base class) enforcing an implementation compliant with the rest of the toolbox:
The constituting parts of each model are:

| Component | Interface                                                               | Description                                                                                                                                            |
|-----------|-------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| Vehicle model | [`VehicleModelInterface`](../core_api/vehicle_dynamics/interfaces.md)   | Provides the vehicle dynamics funciton and the functionality for computing the (normalized) combined acceleration.                                     |
| State dataclass | [`StateInterface`](../core_api/vehicle_dynamics/interfaces.md)          | Dataclass for storing the state.                                                                                                                       |
| Control input dataclass | [`InputInterface`](../core_api/vehicle_dynamics/interfaces.md)          | Dataclass for storing control inputs.                                                                                                                  |
| Disturbance dataclass | [`DisturbanceInterface`](../core_api/vehicle_dynamics/interfaces.md)    | Dataclass for storing disturbances.                                                                                                                    |
| Full state noise dataclass | [`FullStateNoiseInterface`](../core_api/vehicle_dynamics/interfaces.md) | Dataclass for storing sensor noise of the full state feedback sensor (see the [sensor model documentation](../core_api/simulation/sensor_models.md)). |
| Trajectory dataclass | [`TrajectoryInterface`](../core_api/vehicle_dynamics/interfaces.md)     | Dataclass for storing state/input/distubrance trajectories |
| SIDT factory | [`StateInputDisturbanceTrajectoryFactoryInterface`](../core_api/vehicle_dynamics/interfaces.md)                    | Creates states, inputs, disturbances and trajectories for the model. |

All constituting parts inherit from their respective interface. Respecting these interfaces allows users to easily incorporate their own work.

For more details, check the documentation of the [vehicle model interface](../explanations/vehicle_dynamics/interfaces.md).

## Example

Let us integrate the [`KinematicBicycleModel`](../core_api/vehicle_dynamics/kb.md)) as an example for you to follow for your own vehicle model.

### 1. Setup the state dataclass:
We start with the implementation of the state dataclass, whose attributes are the state variables:

```Python3
@dataclass
class KBState(StateInterface):
    """
    Dataclass storing the states of the kinematic bicycle model.
    """

    position_x: float = None
    position_y: float = None
    velocity: float = None
    heading: float = None
    steering_angle: float = None

    @property
    def dim(self) -> int:
        """
        :return: state dimension
        """
        return KBStateIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        x_np = np.zeros((self.dim,))
        x_np[KBStateIndices.position_x] = self.position_x
        x_np[KBStateIndices.position_y] = self.position_y
        x_np[KBStateIndices.velocity] = self.velocity
        x_np[KBStateIndices.heading] = self.heading
        x_np[KBStateIndices.steering_angle] = self.steering_angle

        return x_np
```

The `convert_to_array()` method returns a `numpy` array storing the state variables in its entries.
Since we frequently have to represent the state as an array (e.g., when evaluating the vehicle dynamics function), we implemented another dataclass providing the indices of the entries storing the respective state variable for safe access and conversion.

```python3
@dataclass(frozen=True)
class KBStateIndices(StateInterfaceIndex):
    """
    Indices of the states of the kinematic bicycle model.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4
```
### 2. Setup the control input, disturbance, and full state noise dataclass:

The other point dataclasses follow the same pattern as the state dataclass.
Therefore, we paste the definition of the respective point and point index dataclasses below without further explanation.

```python3
@dataclass(frozen=True)
class KBInputIndices(InputInterfaceIndex):
    """
    Indices of the control inputs of the kinematic bicycle model.
    """

    dim: int = 2
    acceleration: int = 0
    steering_angle_velocity: int = 1


@dataclass()
class KBInput(InputInterface):
    """
    Dataclass storing the control input of the kinematic bicycle model.
    """

    acceleration: float = None
    steering_angle_velocity: float = None

    @property
    def dim(self) -> int:
        """
        :return: control input dimension
        """
        return KBInputIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """

        u_np = np.zeros((self.dim,))
        u_np[KBInputIndices.acceleration] = self.acceleration
        u_np[KBInputIndices.steering_angle_velocity] = self.steering_angle_velocity

        return u_np
```

```python3
@dataclass(frozen=True)
class KBDisturbanceIndices(DisturbanceInterfaceIndex):
    """
    Indices of the disturbances of the kinematic bicycle model.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


@dataclass
class KBDisturbance(DisturbanceInterface):
    """
    Dataclass storing the disturbances of the kinematic bicycle model.
    """

    position_x: float = 0.0
    position_y: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    steering_angle: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: disturbance dimension
        """
        return KBDisturbanceIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[KBDisturbanceIndices.position_x] = self.position_x
        w_np[KBDisturbanceIndices.position_y] = self.position_y
        w_np[KBDisturbanceIndices.velocity] = self.velocity
        w_np[KBDisturbanceIndices.heading] = self.heading
        w_np[KBDisturbanceIndices.steering_angle] = self.steering_angle

        return w_np
```

```python3
@dataclass(frozen=True)
class KBNoiseIndices(FullStateNoiseInterfaceIndex):
    """
    Indices of the noise variables.
    """

    dim: int = 5
    position_x: int = 0
    position_y: int = 1
    velocity: int = 2
    heading: int = 3
    steering_angle: int = 4


@dataclass
class KBNoise(FullStateNoiseInterface):
    """
    Full state noise of the kinematic bicycle model - required for the full state feedback sensor model.
    """

    position_x: float = 0.0
    position_y: float = 0.0
    velocity: float = 0.0
    heading: float = 0.0
    steering_angle: float = 0.0

    @property
    def dim(self) -> int:
        """
        :return: noise dimension
        """
        return KBNoiseIndices.dim

    def convert_to_array(self) -> np.ndarray:
        """
        Converts instance of class to numpy array.
        :return: np.ndarray of dimension (self.dim,)
        """
        w_np = np.zeros((self.dim,))
        w_np[KBNoiseIndices.position_x] = self.position_x
        w_np[KBNoiseIndices.position_y] = self.position_y
        w_np[KBNoiseIndices.velocity] = self.velocity
        w_np[KBNoiseIndices.heading] = self.heading
        w_np[KBNoiseIndices.steering_angle] = self.steering_angle

        return w_np
```

### 3. Setup the trajectory dataclass:

Most of the functionality of the trajectory data class—such as conversion to an array or querying a trajectory point at a discrete time step—is already implemented in the [`TrajectoryInterface`](../core_api/vehicle_dynamics/interfaces.md).

# TODO: what about the conversion to CR obstacles? 

### 4. Setup the SIDT factory

To create instances of point dataclasses from a given array (e.g. when converting the output of the simulation to a state dataclass instance), we require a model-specific implementation of the [`StateInputDisturbanceTrajectoryFactoryInterface`](../core_api/vehicle_dynamics/interfaces.md):

```Python3
class KBSIDTFactory(StateInputDisturbanceTrajectoryFactoryInterface):
    """
    Factory for creating kinematic bicycle model State, Input, Disturbance, and Trajectory.
    """

    state_dimension: int = KBStateIndices.dim
    input_dimension: int = KBInputIndices.dim
    disturbance_dimension: int = KBDisturbanceIndices.dim

    @classmethod
    def state_from_args(
        cls,
        position_x: float,
        position_y: float,
        velocity: float,
        heading: float,
        steering_angle: float,
    ) -> Union["KBState"]:
        """
        Create State from args
        :param position_x: position x of center of gravity (Cartesian coordinates)
        :param position_y: position y of center of gravity (Cartesian coordinates)
        :param velocity: velocity
        :param heading: heading
        :param steering_angle: steering angle
        :return: KBState
        """
        return KBState(
            position_x=position_x,
            position_y=position_y,
            velocity=velocity,
            heading=heading,
            steering_angle=steering_angle,
        )

    @classmethod
    def input_from_args(
        cls,
        acceleration: float,
        steering_angle_velocity,
    ) -> Union["KBInput"]:
        """
        Create Input from args
        :param acceleration: longitudinal acceleration
        :param steering_angle_velocity: steering angle velocity
        :return: KBInput
        """
        return KBInput(
            acceleration=acceleration, steering_angle_velocity=steering_angle_velocity
        )

    @staticmethod
    def disturbance_from_args(
        position_x: float = 0.0,
        position_y: float = 0.0,
        velocity: float = 0.0,
        heading: float = 0.0,
        steering_angle: float = 0.0,
    ) -> Union["KBDisturbance"]:
        """
        Create Disturbance from args - the default value of all variables is zero.
        :param position_x: position x of center of gravity
        :param position_y: position y of center of gravity
        :param velocity: velocity
        :param heading: heading
        :param steering_angle: steering angle
        :return: KBDisturbance
        """
        return KBDisturbance(
            position_x=position_x,
            position_y=position_y,
            velocity=velocity,
            heading=heading,
            steering_angle=steering_angle,
        )

    @classmethod
    def state_from_numpy_array(
        cls,
        x_np: np.ndarray[tuple[float], np.dtype[np.float64]],
    ) -> Union["KBState"]:
        """
        Create State from numpy array
        :param x_np: state vector - array of dimension (cls.state_dimension,)
        :return: KBState
        """

        if x_np.ndim > 1 or x_np.shape[0] != cls.state_dimension:
            logger.error(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.state_dimension},) but is {x_np.ndim}"
            )

        return KBState(
            position_x=x_np[KBStateIndices.position_x],
            position_y=x_np[KBStateIndices.position_y],
            velocity=x_np[KBStateIndices.velocity],
            heading=x_np[KBStateIndices.heading],
            steering_angle=x_np[KBStateIndices.steering_angle],
        )

    @classmethod
    def input_from_numpy_array(
        cls, u_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["KBInput"]:
        """
        Create Input from numpy array
        :param u_np: control input - array of dimension (cls.input_dimension,)
        :return: KBInput
        """

        if u_np.ndim > 1 or u_np.shape[0] != cls.input_dimension:
            logger.error(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.input_dimension},) but is {u_np.ndim}"
            )

        return KBInput(
            acceleration=u_np[KBInputIndices.acceleration],
            steering_angle_velocity=u_np[KBInputIndices.steering_angle_velocity],
        )

    @classmethod
    def disturbance_from_numpy_array(
        cls, w_np: np.ndarray[tuple[float], np.dtype[np.float64]]
    ) -> Union["KBDisturbance"]:
        """
        Create Disturbance from numpy array
        :param w_np: disturbance - array of dimension (cls.disturbance_dimension,)
        :return: KBDisturbance
        """

        if w_np.ndim > 1 or w_np.shape[0] != cls.disturbance_dimension:
            logger.error(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )
            raise ValueError(
                f"Size of np_array should be ({cls.disturbance_dimension},) but is {w_np.shape}"
            )

        return KBDisturbance(
            position_x=w_np[KBDisturbanceIndices.position_x],
            position_y=w_np[KBDisturbanceIndices.position_y],
            velocity=w_np[KBDisturbanceIndices.velocity],
            heading=w_np[KBDisturbanceIndices.heading],
            steering_angle=w_np[KBDisturbanceIndices.steering_angle],
        )

    @classmethod
    def trajectory_from_points(
        cls,
        trajectory_dict: Union[Dict[int, KBState], Dict[int, KBInput]],
        mode: TrajectoryMode,
        t_0: float,
        delta_t: float,
    ) -> "KBTrajectory":
        """
        Create State, Input, or Disturbance Trajectory from list of KB points.
        :param trajectory_dict: dict of time steps to kb points
        :param mode: type of points (State, Input, or Disturbance)
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: KBTrajectory
        """
        return KBTrajectory(points=trajectory_dict, mode=mode, t_0=t_0, delta_t=delta_t)

    @classmethod
    def trajectory_from_numpy_array(
        cls,
        traj_np: np.ndarray[tuple[float, float], np.dtype[np.float64]],
        mode: TrajectoryMode,
        time_steps: List[int],
        t_0: float,
        delta_t: float,
    ) -> "KBTrajectory":
        """
        Create State, Input, or Disturbance Trajectory from numpy array.
        :param traj_np: numpy array storing the values of the point variables
        :param mode: type of points (State, Input, or Disturbance)
        :param time_steps: time steps of the points in the columns of traj_np
        :param t_0: initial time - float
        :param delta_t: sampling time - float
        :return: KBTrajectory
        """

        # convert trajectory points to State/Input/DisturbanceInterface
        points_val = []
        for kk in range(len(time_steps)):
            if mode == TrajectoryMode.State:
                points_val.append(cls.state_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Input:
                points_val.append(cls.input_from_numpy_array(traj_np[:, kk]))
            elif mode == TrajectoryMode.Disturbance:
                points_val.append(cls.disturbance_from_numpy_array(traj_np[:, kk]))

        return KBTrajectory(
            points=dict(zip(time_steps, points_val)),
            mode=mode,
            delta_t=delta_t,
            t_0=t_0,
        )
```

### 5. Setup the vehicle model

As a last component, the vehicle model itself has to be implemented.
Functionality that is shared among vehicle models, such as the time-discretization for MPC or the linearization of the dynamics, is implemented in the [`VehicleModelInterface`](../core_api/vehicle_dynamics/interfaces.md).
All that is left to be implemented are:

```python3
class KinematicBicycle(VehicleModelInterface):
    """
    Kinematic bicycle model.
    Reference point for the vehicle dynamics: center of gravity.
    """

    @classmethod
    def factory_method(
        cls, params: VehicleParameters, delta_t: float
    ) -> "KinematicBicycle":
        """
        Factory method to generate class
        :param params: CommonRoad-Control vehicle parameters
        :param delta_t: sampling time
        :return: instance
        """
        return KinematicBicycle(params=params, delta_t=delta_t)

    def __init__(self, params: VehicleParameters, delta_t: float):

        # set vehicle parameters
        self._l_wb = params.l_wb
        self._l_r = params.l_r
        self._a_long_max = params.a_long_max
        self._a_lat_max = params.a_lat_max

        # init base class
        super().__init__(
            params=params,
            nx=KBStateIndices.dim,
            nu=KBInputIndices.dim,
            nw=KBDisturbanceIndices.dim,
            delta_t=delta_t,
        )
```

... the extraction of the input bounds from the vehicle parameters.

```python3
    def _set_input_bounds(self, params: VehicleParameters) -> Tuple[KBInput, KBInput]:
        """
        Extract input bounds from vehicle parameters and returns them as instances of the Input class.
        :param params: vehicle parameters
        :return: lower and upper bound on the inputs - KBInputs
        """

        # lower bound
        u_lb = KBInput(
            acceleration=-params.a_long_max,
            steering_angle_velocity=-params.steering_angle_velocity_max,
        )

        # upper bound
        u_ub = KBInput(
            acceleration=params.a_long_max,
            steering_angle_velocity=params.steering_angle_velocity_max,
        )

        return u_lb, u_ub
```
... the actual dynamics function.
To ensure modularity, the dynamics function has to be implemented in the method `_dynamics_cas`.
The extraction of the single states and control inputs from the respective vectors demonstrates the efficacy of providing the [`KBStateIndices`](../core_api/vehicle_dynamics/kb.md) and [`KBInputIndices`](../core_api/vehicle_dynamics/kb.md) dataclasses.

```python3
    def _dynamics_cas(
        self,
        x: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        u: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
        w: Union[cas.SX.sym, np.ndarray[tuple[float], np.dtype[np.float64]]],
    ) -> Union[cas.SX.sym, np.array]:
        """
        Continuous-time dynamics function of the vehicle model for simulation and symbolic operations using CasADi.
        :param x: state - CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - CasADi symbolic/ array of dimension (self._nu,)
        :param w: disturbance - CasADi symbolic/ array of dimension (self._nw,)
        :return: dynamics function evaluated at (x,u,w) - CasADi symbolic/ array of dimension (self._nx,)
        """

        # extract state
        v = x[KBStateIndices.velocity]
        psi = x[KBStateIndices.heading]
        delta = x[KBStateIndices.steering_angle]

        # extract control input
        a = u[KBInputIndices.acceleration]
        delta_dot = u[KBInputIndices.steering_angle_velocity]

        # compute slip angle
        beta = cas.atan(cas.tan(delta) * self._l_r / self._l_wb)

        # dynamics
        position_x_dot = v * cas.cos(psi + beta)
        position_y_dot = v * cas.sin(psi + beta)
        velocity_dot = a
        heading_dot = v * cas.sin(beta) / self._l_r
        steering_angle_dot = delta_dot

        f = cas.vertcat(
            position_x_dot,
            position_y_dot,
            velocity_dot,
            heading_dot,
            steering_angle_dot,
        )

        # add disturbances
        f = f + np.reshape(w, shape=(w.size, 1))

        return f
```

... the method `compute_normalized_acceleration`, which computes the normalized longitudinal and lateral acceleration from a given state-control input pair.

```python3
    def compute_normalized_acceleration(
        self,
        x: Union[KBState, cas.SX.sym, np.array],
        u: Union[KBInput, cas.SX.sym, np.array],
    ) -> Tuple[Union[float, cas.SX.sym], Union[float, cas.SX.sym]]:
        """
        Computes the normalized longitudinal and lateral acceleration (w.r.t. the maximum acceleration).
        :param x: state - StateInterface/ CasADi symbolic/ array of dimension (self._nx,)
        :param u: control input - InputInterface/ CasADi symbolic/ array of dimension (self._nu,)
        :return: normalized longitudinal and lateral acceleration - float/ CasADi symbolic
        """

        # extract state
        if isinstance(x, KBState):
            x = x.convert_to_array()
        v = x[KBStateIndices.velocity]
        delta = x[KBStateIndices.steering_angle]

        # compute slip angle
        beta = cas.atan(cas.tan(delta) * self._l_r / self._l_wb)
        # compute yaw rate
        heading_dot = v * cas.sin(beta) / self._l_r

        # extract control input
        if isinstance(u, KBInput):
            u = u.convert_to_array()
        a = u[KBInputIndices.acceleration]

        # normalized acceleration
        a_long_norm = a / self._a_long_max
        a_lat_norm = (v * heading_dot) / self._a_lat_max

        return a_long_norm, a_lat_norm
```
