from dataclasses import dataclass

from commonroad_control.simulation.uncertainty_model.uncertainty_interface import UncertaintyInterface
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_noise import DBNoise
from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters

from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_sidt_factory import DBSIDTFactory
from commonroad_control.vehicle_dynamics.dynamic_bicycle.db_disturbance import DBDisturbance


@dataclass(frozen=True)
class BMW3seriesParams(VehicleParameters):
    """
    Parameters are taken from the package "CommonRoad Vehicle Models and Cost Functions" (vehicle ID: 2)
    " M. Althoff, M. Koschi and S. Manzinger, "CommonRoad: Composable benchmarks for motion planning on roads,"
    IEEE Intelligent Vehicles Symposium, 2017, pp. 719-726"
    """
    l_wb: float = 2.578
    l_f: float = 1.156
    l_r: float = 1.422
    m: float = 1093.0
    I_zz: float = 1791.0
    C_f: float = 20.89 * 1.048
    C_r: float = 20.89 * 1.048
    h_cog: float = 0.574
    a_long_max: float = 11.5 #TODO check values
    a_lat_max: float = 11.5 # TODO check values
    steering_angle_max: float = 1.066
    steering_angle_velocity_max: float = 0.4

    # dynamic bicycle model: parameters of Gaussian disturbance
    disturbance_gaussian_mean: DBDisturbance = DBSIDTFactory.disturbance_from_args()
    disturbance_gaussian_std: DBDisturbance = DBSIDTFactory.disturbance_from_args(
        velocity_long=0.8,
        velocity_lat=0.35,
        yaw_rate=0.035
    )

    # full state feedback for dynamic bicycle model: parameters of Gaussian noise
    noise_gaussian_mean: UncertaintyInterface = DBNoise()
    noise_gaussian_std: UncertaintyInterface = DBNoise(
        position_x=0.075,
        position_y=0.075,
    )

