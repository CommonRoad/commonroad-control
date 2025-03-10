from abc import ABC
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParameters(ABC):
    """
    Vehicle parameters.
    """
    l_wb: float  # wheelbase
    l_f: float  # distance front-axle to center of gravity
    l_r: float  # distance rear-axle to center of gravity
    m: float  # mass
    I_zz: float  # moment of inertia around the vertical axis
    C_f: float  # front tyre cornering stiffness
    C_r: float  # rear tyre cornering stiffness
    h_cog: float  # height of center of gravity
    g: float = 9.81  # gravitational acceleration
