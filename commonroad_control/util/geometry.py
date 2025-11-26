import logging

import numpy as np
from shapely import LineString, Point

logger = logging.getLogger(__name__)


def signed_distance_point_to_linestring(point: Point, linestring: LineString) -> float:
    """
    Signed distance between a shapely point and shapely linestring. Positive means left of line vector
    :param point: shapely point
    :param linestring: shapely linestring
    :return: signed distance, positive meaning left of linestring segment vector
    """

    distance: float = linestring.distance(point)
    coords = list(linestring.coords)
    arclength = linestring.project(point)
    point_on_line = linestring.interpolate(arclength)
    point_before_line = None
    for i in range(len(coords) - 1):
        seg_start = Point(coords[i])
        seg_end = Point(coords[i + 1])
        if (
            linestring.project(seg_start) <= arclength
            and linestring.project(seg_end) >= arclength
        ):
            point_before_line = seg_start
            break

    if point_before_line is None:
        # TODO: Robustify
        logger.error("point before line is none")
        raise ValueError("point before line is none")

    vector_line: np.ndarray = np.asarray(
        [point_on_line.x - point_before_line.x, point_on_line.y - point_before_line.y]
    )
    vector_point: np.ndarray = np.asarray(
        [point.x - point_on_line.x, point.y - point_on_line.y]
    )
    cross_product = np.cross(vector_line, vector_point)

    return distance if cross_product >= 0.0 else -distance
