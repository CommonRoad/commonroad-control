from typing import List, Dict
import numpy as np

from commonroad_control.optimal_control.ocp_dataclasses import State
from commonroad_control.optimal_control.collision_avoidance.ca_polyhedron import Polyhedron
from commonroad_control.optimal_control.collision_avoidance.utils import Vertex


class ConstraintFactory:
    def __init__(self, N: int, nx: int):
        self._polyhedra = dict.fromkeys(range(N+1), [])
        self._nx = nx
        self._N = N

    def set_polyhedral_obstacles(self, polyhedra: Dict[int, List[Polyhedron]]):
        self._polyhedra = polyhedra

    def polyhedral_obstacles_at_time_step(self, kk: int) -> List[Polyhedron]:
        return self._polyhedra[kk]

    def project_and_linearize_at_time_step(self, kk: int, x: State) -> [np.array, np.array]:
        con_mat = []
        con_rhs = []

        for p in self.polyhedral_obstacles_at_time_step(kk):
            [p_lin, _] = p.project_and_linearize(x)
            con_mat.append(p_lin.A)
            con_rhs.append(p_lin.b)

        con_mat_xy = np.vstack(con_mat)
        con_mat = np.zeros((len(self._polyhedra[kk]), self._nx))
        con_mat[:, x.idx_position_x] = con_mat_xy[:, 0]
        con_mat[:, x.idx_position_y] = con_mat_xy[:, 1]
        con_rhs = np.hstack(con_rhs)
        return con_mat, con_rhs
