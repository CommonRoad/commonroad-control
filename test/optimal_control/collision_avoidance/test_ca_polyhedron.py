from math import sqrt
import numpy as np

from commonroad_control.optimal_control.ocp_dataclasses import State
from commonroad_control.optimal_control.collision_avoidance.ca_polyhedron import Polyhedron, Vertex


def test_ca_polyhedron() -> bool:

    res = np.full((4, 1), fill_value=False)

    # simple test - create a polyhedron that contains all points with y <= 0
    x1 = [0, 0.5]
    v1 = Vertex(x=x1[0], y=x1[1])
    x2 = [2, 0.5]
    v2 = Vertex(x=x2[0], y=x2[1])
    poly1 = Polyhedron(cand_vertices=[v1, v2], passing_direction=1)

    # ... state that is contained in poly1
    x_in = [1, -1]
    state_in = State(position_x=x_in[0], position_y=x_in[1])
    res_cont_in = poly1.contains(state_in)
    # ... project and linearize at x_in
    [hs, projection_state_in] = poly1.project_and_linearize(state_in)
    # ... check results
    if (res_cont_in and (hs.A.dot(x_in) >= hs.b)
            and np.linalg.norm(projection_state_in.project_position() - np.array([1, 0.5])) <= 1e-16
            and poly1.contains(projection_state_in)):
        res[0] = True

    # ... state that is not contained in poly1
    x_out = [3, 1]
    state_out = State(position_x=x_out[0], position_y=x_out[1])
    # ... check containment
    res_cont_out = poly1.contains(state_out)
    # ... project and linearize at x_out
    [hs, projection_state_out] = poly1.project_and_linearize(state_out)
    # ... check results
    if ((not res_cont_out) and (hs.A.dot(x_out) <= hs.b)
            and np.linalg.norm(projection_state_out.project_position() - np.array([3, 0.5])) <= 1e-6)\
            and poly1.contains(projection_state_out):
        res[1] = True

    # slightly more complex test
    x3 = [3, 1.5]
    v3 = Vertex(x=x3[0], y=x3[1])
    poly2 = Polyhedron(cand_vertices=[v1, v2, v3], passing_direction=-1)
    # ... state that is contained in poly2
    x_in = [2, 1.1]
    state_in = State(position_x=x_in[0], position_y=x_in[1])
    res_cont_in = poly2.contains(state_in)
    # ... project and linearize at x_in
    [hs, projection_state_in] = poly2.project_and_linearize(state_in)
    # ... check results
    if (res_cont_in and (hs.A.dot(x_in) >= hs.b)
            and np.linalg.norm(hs.A - [-sqrt(2)/2, sqrt(2)/2]) <= 1e-6
            and poly2.contains(projection_state_in)):
        res[2] = True

    # ... state that is not contained in poly2
    x_out = [2.2, -0.5]
    state_out = State(position_x=x_out[0], position_y=x_out[1])
    # ... check containment
    res_cont_out = poly2.contains(state_out)
    # ... project and linearize at x_out
    [hs, projection_state_out] = poly2.project_and_linearize(state_out)
    # ... check results
    # -> compute expected normal vector (convex interpolation between [0, -1] and [sqrt(2)/2, -sqrt(2)/2]
    n1 = [0, -1]
    n2 = [sqrt(2)/2, -sqrt(2)/2]
    # TODO: replace check below by containment in convex cone
    # alpha_x = (-hs.A[0] - n2[0])/(n1[0]-n2[0])
    # alpha_y = (-hs.A[1] - n2[1])/(n1[1]-n2[1])
    if (not res_cont_out and (hs.A.dot(x_out) <= hs.b)
            and np.linalg.norm(projection_state_out.project_position() - np.array([2, 0.5])) <= 1e-6
            and poly2.contains(projection_state_out)):
        res[3] = True

    # # check emptiness
    # # ... empty polyhedron
    # x3_e = [2, 1]
    # v3_e = Vertex(x=x3_e[0], y=x3_e[1])
    # x_4e = [0, 1]
    # v4_e = Vertex(x=x_4e[0], y=x_4e[1])
    # poly_e = Polyhedron(cand_vertices=[[v1, v2], [v3_e, v4_e]], passing_direction=1)
    # # ... non-empty polhyedron
    # x3_ne = [0, 1]
    # v3_ne = Vertex(x=x3_ne[0], y=x3_ne[1])
    # x_4ne = [2, 1]
    # v4_ne = Vertex(x=x_4ne[0], y=x_4ne[1])
    # poly_ne = Polyhedron(cand_vertices=[v1, v2, v3_ne, v4_ne], passing_direction=1)
    # # ... check results
    # if poly_e.is_empty() and not poly_ne.is_empty():
    #     res[4] = True

    return all(res)


if __name__ == '__main__':
    res = test_ca_polyhedron()
