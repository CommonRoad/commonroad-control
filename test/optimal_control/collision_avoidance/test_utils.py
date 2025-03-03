import numpy as np
import clarabel
from typing import List
from scipy import sparse

from commonroad_control.optimal_control.collision_avoidance.utils import monotone_chain, Vertex


def test_monotone_chain() -> bool:

    def _convex_combination(x: Vertex, conv_hull: List[Vertex]) -> (bool, np.array):

        num_vertices = len(conv_hull)
        P = sparse.csc_matrix(np.zeros((num_vertices, num_vertices)))  # (quadratic) cost matrix
        q = np.ones(num_vertices)
        # constraints: x is a convex combination of the vertices
        # ... inequality constraint: sum_i theta_i*vertex_i = x
        # .... sum_i theta_i = 1
        # .... theta_i >= 0
        A = np.vstack([np.array([v.convert_to_array() for v in conv_hull]).T, np.ones((1, num_vertices)),
                      -np.identity(num_vertices)])
        A = sparse.csc_matrix(A)
        b = np.hstack([x.convert_to_array(), 1, np.zeros(num_vertices)])
        clarabel_cones = [clarabel.ZeroConeT(2 + 1), clarabel.NonnegativeConeT(len(conv_hull))]
        solver = clarabel.DefaultSolver(P, q, A, b, clarabel_cones, clarabel.DefaultSettings())
        solution = solver.solve()
        x_cc = np.array([v.convert_to_array() for v in conv_hull]).T.dot(solution._x)

        # problem is primal infeasible if the candidate vertex is not contained in the convex hull
        return False if str(solution.status) == 'PrimalInfeasible' else True, x_cc

    res = np.full((3, 1), fill_value=True)

    # test 1: list of candidate vertices is the convex hull
    cand_vertices_t1 = [Vertex(x=0, y=0), Vertex(x=1, y=0.5), Vertex(x=2, y= 0.7), Vertex(x=4, y=0.1)]
    conv_hull_t1 = monotone_chain(cand_vertices_t1)
    for v1, v2 in zip(cand_vertices_t1, conv_hull_t1):
        if np.linalg.norm(v1.convert_to_array() - v2.convert_to_array()) > 1e-16:
            res[0] = False

    # test 2: last vertex is removed
    redundant_vertex = Vertex(x=1.5, y=0.6)
    cand_vertices_t2 = cand_vertices_t1
    cand_vertices_t2.append(redundant_vertex)
    conv_hull_t2 = monotone_chain(cand_vertices_t2)
    [is_feasible, redundant_vertex_cc] = _convex_combination(redundant_vertex, conv_hull_t2)
    if not (is_feasible and np.linalg.norm(redundant_vertex.convert_to_array() - redundant_vertex_cc) <= 1e-8):
        res[1] = False

    # test 3: random testing
    num_vertices = 10
    # ... sample random vertices
    rand_vertices = 10*np.random.rand(num_vertices, 2)
    cand_vertices_t3 = [Vertex(x=rand_vertices[ii, 0], y=rand_vertices[ii, 1]) for ii in range(num_vertices)]
    # ... add vertices at (smallest x-coordinate, smallest y-coordinate), (largest x-coordinate, smallest y-coordinate)
    # since monotone_chain only computes the "upper part" of the convex hull
    x_min = min(rand_vertices[:, 0])
    x_max = max(rand_vertices[:, 0])
    y_min = min(rand_vertices[:, 1])
    cand_vertices_t3.append(Vertex(x=x_min-0.1, y=y_min-0.1))
    cand_vertices_t3.append(Vertex(x=x_max+0.1, y=y_min-0.1))
    # ... compute convex hull
    conv_hull_t3 = monotone_chain(cand_vertices_t3)
    # ... check containment of vertices in convex hull
    for v in cand_vertices_t3:
        [is_feasible, redundant_vertex_cc] = _convex_combination(v, conv_hull_t3)
        if not (is_feasible and np.linalg.norm(v.convert_to_array() - redundant_vertex_cc) <= 1e-8):
            res[2] = False

    return all(res)


if __name__ == '__main__':
    res = test_monotone_chain()
