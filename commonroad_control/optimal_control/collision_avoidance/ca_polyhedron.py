import numpy as np
import clarabel
from scipy import sparse
from math import inf
from typing import List
import matplotlib.pyplot as plt

from commonroad_control.optimal_control.ocp_dataclasses import State
from commonroad_control.optimal_control.collision_avoidance.utils import monotone_chain, Vertex, HalfSpace


class Polyhedron:
    def __init__(self, cand_vertices: List[Vertex], passing_direction: int):
        """
        Given pairs of points that support the half-spaces of the polyhedron, its halfspace representation is computed
        and the parameters of the projection problem (see project_and_linearize) are set.
        :param cand_vertices: list of (candidate) vertices of the polyhedron
        :param passing_direction: orientation of the support vector - "1" if the polyhedron has to be passed on the
        left, "-1" if it has to be passed on the right
        """

        # tolerance for checking containment
        self._tol = 1e-8

        # check inputs
        if passing_direction not in {1, -1}:
            raise ValueError("Input argument passing_direction has to be either 1 or -1")

        # compute convex hull of vertices
        # ... if passing direction is -1, mirror vertices w.r.t. the horizontal axis before computing the convex hull
        cand_vertices = [Vertex(v.x, passing_direction*v.y) for v in cand_vertices]
        # ... compute the convex hull
        vertices = monotone_chain(cand_vertices)
        # ... if passing direction is -1, flip back vertices
        self._vertices = [Vertex(v.x, passing_direction*v.y) for v in vertices]

        # TODO: include tapering of convex hull

        # convert into half-space representation
        self._A = np.zeros((len(self._vertices) - 1, 2), dtype=np.float64)
        # self._b = np.zeros((len(hs_support_points), 1), dtype=np.float64)
        self._b = np.zeros(len(self._vertices) - 1, dtype=np.float64)
        # set oriented vector perpendicular to xy-plane (normal vector of half-space must face outwards, i.e.,
        # x in P <=> self._A*x <= self._b
        z_vector = np.array([0, 0, passing_direction])
        for ii in range(len(self._vertices) - 1):
            # linear part of the half-space - sort support points by ascending x-coordinate to avoid empty sets
            if abs(self._vertices[ii+1].x - self._vertices[ii].x) < self._tol:
                raise ValueError("Vertically aligned support points are not supported.")
            hs_direction = np.hstack((self._vertices[ii+1].convert_to_array() - self._vertices[ii].convert_to_array(), 0))

            tmp = np.cross(z_vector, hs_direction)
            self._A[ii, :] = tmp[0:2]/np.linalg.norm(tmp[0:2])
            self._b[ii] = self._A[ii, :].dot(self._vertices[ii].convert_to_array())

        # parameters for the projection problem using clarabel as the QP solver
        self._clarabel_P = 2*sparse.identity(2, format='csc')  # (quadratic) cost matrix
        self._clarabel_A = sparse.csc_matrix(self._A)
        self._clarabel_cones = [clarabel.NonnegativeConeT(self._b.size)]  # only inequality constraints

        # # post-processing: check whether polyhedron is empty or not
        # self._is_empty = self.is_empty()
    #
    # def is_empty(self) -> bool:
    #     """
    #     Solves a quadratic program using clarabel to check whether the polytope is empty or not.
    #     :return: true, if the polytope is empty - false, otherwise
    #     """
    #
    #     solver = clarabel.DefaultSolver(sparse.identity(2, format='csc'), np.zeros(2),
    #                                     self._clarabel_A, self._b, self._clarabel_cones, clarabel.DefaultSettings())
    #     solution = solver.solve()
    #     if str(solution.status) == 'PrimalInfeasible':
    #         return True
    #     else:
    #         return False

    def contains(self, x: State) -> bool:
        """
        Checks whether the current position of the vehicle is contained in the polyhedron.
        :param x: current state of the vehicle
        :return: true, if the position is contained - false, otherwise
        """
        if all(self._A.dot(x.project_position()) <= self._b + self._tol):
            return True
        else:
            return False

    def project_and_linearize(self, x: State) -> (HalfSpace, State):
        """
        Projects the current position of the vehicle onto the boundary of the polyhedron and returns a linearized
        collision avoidance constraints
                A*x.project_position() <= b.
        The projection of the current position of the vehicle is obtained by solving the following quadratic program
                min     np.linalg.norm(y-x.project_position())**2
                s.t.    self._A*y <= self._b,
        which can be encoded as
                min     0.5*y.T*self._clarabel_P*y + x.project_position().T*y
                s.t.    self._clarabel_A*y + s = self._b,
                        s >= 0,
        see the clarabel webpage https://clarabel.org/stable/ for more information.
        If the current position is contained in the interior of the polyhedron, solving this QP yields an interior point
        of the polyhedron. To obtain a point on the boundary, projects the position onto each half-space of the
        polyhedron to detect the closest one since this half-space returns the closest point on the boundary.
        :param x:   state of the vehicle
        :return:    half-space representing the linearized constraint, projection of x
        """
        position = x.project_position()

        # compute projection of position onto the boundary of the polyhedron
        if all(self._A.dot(position) <= self._b + self._tol):
            # position is contained in the polyhedron (the polyhedron is enlarged for this check to avoid numerical
            # issues)
            # iterate over all half-spaces and find the one that is closest to position
            dist_min = inf
            idx = 0
            for ii in range(self._b.size):
                dist = self._b[ii] - self._A[ii, :].dot(position)
                if dist < dist_min:
                    dist_min = dist
                    idx = ii

            # compute projection point
            s = dist_min/np.linalg.norm(self._A[idx, :])**2
            position_projected = position + s*self._A[idx, :]

            # linearize
            mat_lin = -self._A[idx, :]

        else:
            # position is not contained in the polyhedron

            # solve quadratic program to project position onto the boundary of the polyhedron
            clarabel_q = -2*position.T  # coefficients of the linear term
            solver = clarabel.DefaultSolver(self._clarabel_P, clarabel_q, self._clarabel_A, self._b,
                                            self._clarabel_cones, clarabel.DefaultSettings())
            solution = solver.solve()
            position_projected = solution.x

            # linearize
            mat_lin = position_projected - position
            mat_lin = mat_lin / np.linalg.norm(mat_lin)

        # right-hand side of the linearized collision avoidance constraint
        rhs_lin = mat_lin.dot(position_projected)

        # projected state
        x_proj = State(position_x=position_projected[0], position_y=position_projected[1])

        return HalfSpace(A=mat_lin, b=rhs_lin), x_proj

    def plot(self):
        np_vertices = np.vstack([v.convert_to_array() for v in self._vertices])
        plt.plot(np_vertices[:, 0], np_vertices[:, 1], 'bo-')
