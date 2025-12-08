import unittest

import numpy as np

from commonroad_control.vehicle_dynamics.double_integrator.double_integrator import DoubleIntegrator
from commonroad_control.vehicle_parameters.BMW3series import BMW3seriesParams
from commonroad_control.vehicle_dynamics.double_integrator.di_state import DIState, DIStateIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_input import DIInput, DIInputIndices
from commonroad_control.vehicle_dynamics.double_integrator.di_sidt_factory import DISIDTFactory
from commonroad_control.vehicle_dynamics.utils import TrajectoryMode

from commonroad_control.control.model_predictive_control.model_predictive_control import ModelPredictiveControl
from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx, SCvxParameters

from commonroad_control.simulation.simulation.simulation import Simulation


class TestModelPredictivecontrol(unittest.TestCase):
    """
    Tests model predictive control.
    """

    def test_di_control2origin(self) -> None:
        """
        Simple test case: steer the double-integrator system to the origin.
        :return:
        """

        # time
        horizon_ocp = 10
        horizon_sim = 10
        delta_t = 0.1

        # instantiate double integrator model and sit factory
        vehicle_model = DoubleIntegrator(
            params=BMW3seriesParams(),
            delta_t=delta_t)
        sit_factory = DISIDTFactory()

        # cost matrices
        cost_xx = np.eye(DIStateIndices.dim)
        cost_uu = 0.01*np.eye(DIInputIndices.dim)
        cost_final = np.eye(DIStateIndices.dim)

        # instantiate optimal control problem solver (SCvx)
        solver_parameters = SCvxParameters(max_iterations=1)
        scvx_solver = OptimalControlSCvx(
            vehicle_model=vehicle_model,
            sit_factory=sit_factory,
            horizon=horizon_ocp,
            delta_t=delta_t,
            cost_xx = cost_xx,
            cost_uu = cost_uu,
            cost_final=cost_final,
            ocp_parameters=solver_parameters
        )

        # instantiate model predictive controller
        mpc = ModelPredictiveControl(
            ocp_solver=scvx_solver
        )

        # setup simulator for closed-loop simulation
        simulator =  Simulation(
            vehicle_model=vehicle_model,
            state_input_factory=sit_factory)

        # reference trajectory for tracking
        x_ref = np.zeros((DIStateIndices.dim,horizon_sim + horizon_ocp + 1))
        u_ref = np.zeros((DIInputIndices.dim,horizon_sim + horizon_ocp))

        # initial state
        x0 = DIState(
            position_long=5.0,
            position_lat=5.0,
            velocity_long=0.0,
            velocity_lat=0.0
        )

        # closed-loop simulation
        x_sim = {0: x0}
        u_sim = {}
        for kk in range(horizon_sim):
            # extract reference trajectory and create trajectory interface object
            x_ref_np = x_ref[:, kk:kk+horizon_ocp+1]
            u_ref_np = u_ref[:, kk:kk+horizon_ocp]
            tmp_x_ref = sit_factory.trajectory_from_numpy_array(
                    traj_np=x_ref_np,
                    mode=TrajectoryMode.State,
                    time_steps=[kk for kk in range(horizon_ocp+1)],
                    t_0=kk*delta_t,
                    delta_t=delta_t
            )
            tmp_u_ref = sit_factory.trajectory_from_numpy_array(
                    traj_np=u_ref_np,
                    mode=TrajectoryMode.Input,
                    time_steps=[kk for kk in range(horizon_ocp)],
                    t_0=kk*delta_t,
                    delta_t=delta_t
            )

            # solve optimal control problem
            u_opt = mpc.compute_control_input(
                x0=x_sim[kk],
                x_ref=tmp_x_ref,
                u_ref=tmp_u_ref
            )

            # simulate system
            _, _, x_nominal = simulator.simulate(
                x0=x_sim[kk],
                u=u_opt,
                t_final=delta_t
            )

            # store solution
            x_sim[kk+1] = x_nominal.final_point
            u_sim[kk] = u_opt

