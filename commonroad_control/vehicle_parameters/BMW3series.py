from commonroad_control.vehicle_parameters.vehicle_parameters import VehicleParameters
from dataclasses import dataclass


@dataclass(frozen=True)
class BMW3seriesParams(VehicleParameters):
    """
    Parameters are taken from the package "CommonRoad Vehicle Models and Cost Functions" (vehicle ID: 2)
    " M. Althoff, M. Koschi and S. Manzinger, "CommonRoad: Composable benchmarks for motion planning on roads,"
    IEEE Intelligent Vehicles Symposium, 2017, pp. 719-726"
    """

    l_wb = 2.578
    l_f = 1.156
    l_r = 1.422
    m = 1093.0
    I_zz = 1791.0
    C_f = 20.89
    C_r = 20.89

    def __post_init__(self):
        super().__init__(
            l_wb=self.l_wb,
            l_f=self.l_f,
            l_r=self.l_r,
            m=self.m,
            I_zz=self.I_zz,
            C_f=self.C_f,
            C_R=self.C_r
        )
