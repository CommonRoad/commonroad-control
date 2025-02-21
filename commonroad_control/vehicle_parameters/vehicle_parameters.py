from abc import ABC
from dataclasses import dataclass


@dataclass(frozen=True)
class VehicleParameters(ABC):
    """
    Vehicle parameters.
    """
    l_wb: float  # wheelbase
    l_r: float  # distance rear-axle to center of gravity
    m: float  # mass
    I_zz: float  # moment of inertia around the vertical axis

