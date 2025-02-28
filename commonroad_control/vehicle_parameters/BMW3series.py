from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from dataclasses import dataclass


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
    C_f: float = 20.89
    C_r: float = 20.89