from typing import List
import numpy as np
from dataclasses import dataclass


@dataclass
class Vertex:
    x: float = None
    y: float = None

    def convert_to_array(self) -> np.array:
        x_np = np.zeros(2, dtype=float)
        x_np[0] = self.x
        x_np[1] = self.y
        return x_np


@dataclass
class HalfSpace:
    A: np.zeros(shape=(1, 2))
    b: np.zeros(shape=(1, 1))


def monotone_chain(cand_vertices: List[Vertex]) -> List[Vertex]:
    """
    Implements Andrew's monotone chain algorithm, which has complexity O(n log n), where n denotes the number of points.
    https://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain#Python
    For formulating collision avoidance constraints, we are only interested in the upper hull and, therefore, the
    computation of the lower hull is omitted.
    :param cand_vertices: list of candidate vertices of the convex hull.
    :return: list of vertices of the convex hull in clockwise order, starting from the vertex with the lexicographically
    smallest x-coordinate.
    """

    def _cross(o: np.array, a: np.array, b: np.array):
        # Returns a positive value, if OAB makes a counter-clockwise turn,
        # negative for clockwise turn, and zero if the points are collinear.
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Sort the points lexicographically (tuples are compared lexicographically) and remove duplicates
    cand_vertices = [v.convert_to_array() for v in cand_vertices]
    cand_vertices = sorted(set(map(tuple,cand_vertices)))
    # Build upper hull
    upper = []
    for v in cand_vertices:
        while len(upper) >= 2 and _cross(upper[-2], upper[-1], v) >= 0:
            upper.pop()
        upper.append(v)

    # represent convex hull as sorted list of Vertex objects
    vertices = [Vertex(x=p[0], y=p[1]) for p in upper]
    return vertices
