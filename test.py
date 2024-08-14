import sys
from typing import Type
import numpy as np
from optimalControl.ocp_dataclasses import State
from constrainedIntegrator.constrainedIntegrator_probdef import StateConstrainedIntegrator
from optimalControl.utils import TrajectoryNumpyConverter


def str_to_class(classname):
    return getattr(sys.modules[__name__], classname)(dim=2)


def compute_state(cls: Type[State]) -> np.array:
    return cls.convert_to_array()


dc = str_to_class('State')
dc.position_x = 2
a = dc.position_x
print(dc.position_x)
print(dc)

dc2npidx_dict = {'position_x': 0, 'position_y': 1}
converter = TrajectoryNumpyConverter(dc2npidx_dict,dc_type='State')

x = State(position_x=0.1, position_y=0.2)

x_np = converter.convert_2_numpy(x)
x_back = converter.convert_from_numpy(x_np)

x = StateConstrainedIntegrator(position_x=0.1, position_y=0.3, orientation=0.1, velocity_x=0.0, velocity_y=0.1, velocity_orientation=0.2)

y = x.convert_to_array()
y_np = compute_state(x)

y2 = StateConstrainedIntegrator()
y2.set_values_from_array(y_np)

print(y2)

# todo: test trajectory+asdf
