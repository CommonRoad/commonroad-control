import enum

from commonroad_control.control.model_predictive_control.optimal_control.optimal_control_scvx import OptimalControlSCvx


@enum.unique
class ImplementedOCPSolvers(enum.Enum):
    SCvx = "SCvx"






