import unittest

import numpy as np
import scipy as sp

from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx, SCvxParameters

from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_sit_factory import DISITFactory

from commonroad_control.vehicle_dynamics.kinematic_bicycle.kinematic_bicycle import KinematicBicycle
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_state import KBState, KBStateIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_input import KBInput, KBInputIndices
from commonroad_control.vehicle_dynamics.kinematic_bicycle.kb_sit_factory import KBSITFactory


class TestOptimalControlSCvx(unittest.TestCase):
    """
    Tests successive convexification algorithm for nonconvex optimal control.
    """

    def test_convex_ocp(self) -> None:
        """
        Simple test case: steer the double-integrator system to the orign within one step starting from a state inside
        its one-step zero controllable set.
        :return:
        """
        # instantiate double integrator model
        vehicle_model = DoubleIntegrator(params=BMW3seriesParams(), delta_t=0.1)

        # time
        horizon = 20
        delta_t = 0.1

        # cost matrices
        cost_xx = np.zeros((DIStateIndices.dim, DIStateIndices.dim))
        cost_uu = np.zeros((DIInputIndices.dim, DIInputIndices.dim))
        cost_final = 100*np.eye(DIStateIndices.dim)

        # Compute initial state: can be steered to the origin for t = delta_t*horizon
        u = -1.0
        v = (-delta_t)*u
        pos = 0.5*u*(delta_t*horizon)**2 #'(-0.5 * (delta_t * horizon) ** 2 + (delta_t * horizon) ** 2) * u
        x0 = DIState(position_long=pos, velocity_long=v, position_lat=pos, velocity_lat=v)

        time_state = [kk for kk in range(horizon+1)]
        time_input = [kk for kk in range(horizon)]
        sit_factory = DISITFactory()

        # reference trajectory
        x_ref_np = np.zeros((DIStateIndices.dim, horizon+1))
        x_ref = sit_factory.trajectory_from_numpy_array(
            traj_np=x_ref_np,
            mode=TrajectoryMode.State,
            time_steps=time_state,
            t_0=0,
            delta_t=delta_t
        )
        u_ref_np = np.zeros((DIInputIndices.dim, horizon))
        u_ref = sit_factory.trajectory_from_numpy_array(
            traj_np=u_ref_np,
            mode=TrajectoryMode.Input,
            time_steps=time_input,
            t_0=0,
            delta_t=delta_t
        )

        # initial guess/ reference for linearization
        x_init_np = np.zeros((DIStateIndices.dim, horizon+1))
        x_init = sit_factory.trajectory_from_numpy_array(
            traj_np=x_init_np,
            mode=TrajectoryMode.State,
            time_steps=time_state,
            t_0=0,
            delta_t=delta_t
        )
        u_init_np = np.zeros((DIInputIndices.dim, horizon))
        u_init = sit_factory.trajectory_from_numpy_array(
            traj_np=u_init_np,
            mode=TrajectoryMode.Input,
            time_steps=time_input,
            t_0=0,
            delta_t=delta_t
        )

        # solver parameters
        ocp_parameters = SCvxParameters(max_iterations=1,
                                        soft_tr_penalty_weight=0.0)

        # instantiate model predictive control
        solver = OptimalControlSCvx(vehicle_model=vehicle_model,
                                    sit_factory=sit_factory,
                                    horizon=horizon,
                                    delta_t=delta_t,
                                    cost_xx=cost_xx,
                                    cost_uu=cost_uu,
                                    cost_final=cost_final,
                                    ocp_parameters=ocp_parameters)

        # Solve the optimal control problem
        x_sol, u_sol, iteration_history = solver.solve(x0=x0,
                                                       x_ref=x_ref,
                                                       u_ref=u_ref,
                                                       x_init=x_init,
                                                       u_init=u_init)

        # check result
        self.assertLess(np.linalg.norm(x_sol.final_point.convert_to_array()), 1e-5)

    def test_nonconvex_ocp(self) -> None:
        """
        Nonconvex optimal control: the task is to perform a turn-right manoeuvre. Since we consider a kinematic bicycle
        model the optimal control problem is non-convex.
        :return:
        """

        # instantiate double integrator model
        vehicle_model = KinematicBicycle(params=BMW3seriesParams(), delta_t=0.1)

        # time
        horizon = 10
        delta_t = 0.1

        # cost matrices
        cost_xx = np.zeros((KBStateIndices.dim, KBStateIndices.dim))
        cost_uu = 0*np.eye(KBInputIndices.dim)
        # ... penalize deviation from desired final state
        weights = 10*np.ones((KBStateIndices.dim,))
        weights[KBStateIndices.steering_angle] = 0
        cost_final = np.diag(weights)

        # initial state
        x0 = KBState(position_x=0.0,
                     position_y=0.0,
                     velocity=20.0,
                     heading=0.0,
                     steering_angle=0.0)

        # desired final state
        xf = KBState(position_x=20,
                     position_y=-1.99,
                     velocity=19.87,
                     heading=-0.2,
                     steering_angle=0.0)

        time_state = [kk for kk in range(horizon+1)]
        time_input = [kk for kk in range(horizon)]
        sit_factory = KBSITFactory()

        # reference trajectory
        # ... goal is to reach the target state while minimizing control effort
        x_ref_np = np.zeros((KBStateIndices.dim, horizon + 1))
        x_ref_np[:,-1] = xf.convert_to_array()
        x_ref = sit_factory.trajectory_from_numpy_array(
            traj_np=x_ref_np,
            mode=TrajectoryMode.State,
            time_steps=time_state,
            t_0=0,
            delta_t=delta_t
        )
        u_ref_np = np.zeros((KBInputIndices.dim, horizon))
        u_ref = sit_factory.trajectory_from_numpy_array(
            traj_np=u_ref_np,
            mode=TrajectoryMode.Input,
            time_steps=time_input,
            t_0=0,
            delta_t=delta_t
        )

        # initial guess/ reference for linearization
        # ... state: linear interpolation between x0 and xf
        x_init_interp_fun = sp.interpolate.interp1d(
            [0.0, 1.0],
            np.hstack((np.reshape(x0.convert_to_array(), (KBStateIndices.dim, 1)),
            np.reshape(xf.convert_to_array().transpose(), (KBStateIndices.dim, 1)))),
            kind="linear"
        )
        x_init_np = np.zeros((KBStateIndices.dim, horizon + 1))
        for kk in range(horizon + 1):
            x_init_np[:,kk] = x_init_interp_fun(kk / horizon)
        x_init = sit_factory.trajectory_from_numpy_array(
            traj_np=x_init_np,
            mode=TrajectoryMode.State,
            time_steps=time_state,
            t_0=0,
            delta_t=delta_t
        )
        # ... control inputs: set to zero
        u_init_np = np.zeros((KBInputIndices.dim, horizon))
        u_init = sit_factory.trajectory_from_numpy_array(
            traj_np=u_init_np,
            mode=TrajectoryMode.Input,
            time_steps=time_input,
            t_0=0,
            delta_t=delta_t
        )

        # solver parameters
        ocp_parameters = SCvxParameters(
            max_iterations=20,
            soft_tr_penalty_weight=0.001
        )

        # instantiate model predictive control
        solver = OptimalControlSCvx(vehicle_model=vehicle_model,
                                    sit_factory=sit_factory,
                                    horizon=horizon,
                                    delta_t=delta_t,
                                    cost_xx=cost_xx,
                                    cost_uu=cost_uu,
                                    cost_final=cost_final,
                                    ocp_parameters=ocp_parameters)

        # Solve the optimal control problem
        x_sol, u_sol, iteration_history = solver.solve(x0=x0,
                                                       x_ref=x_ref,
                                                       u_ref=u_ref,
                                                       x_init=x_init,
                                                       u_init=u_init)

        # check result
        self.assertLess(np.linalg.norm(
            np.vstack((x_sol.final_point.position_x - xf.position_x,
                       x_sol.final_point.position_y - xf.position_y,
                       x_sol.final_point.heading - xf.heading,
                       x_sol.final_point.velocity - xf.velocity))),1e-5)



