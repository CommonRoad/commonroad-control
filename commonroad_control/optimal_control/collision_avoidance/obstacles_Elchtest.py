from typing import Dict, List

from commonroad_control.optimal_control.collision_avoidance.ca_polyhedron import Polyhedron
from commonroad_control.optimal_control.collision_avoidance.utils import Vertex


def get_obstacles(w: float, N: int) -> Dict[int, List[Polyhedron]]:

    # track dimensions of the double lane change maneuver according to ISO 3888-2:2011(E)
    # https://cdn.standards.iteh.ai/samples/57253/5e8cdcc7c2fe4c0c8fe6f1de8beb5dba/ISO-3888-2-2011.pdf
    section_1_l = 12.0
    section_1_sw = 0.5*(1.1*w + 0.25)  # semi-width of lane
    section_2_l = 13.5
    section_3_l = 11.0
    section_3_sw = 0.5*(w + 1)  # semi-width of lane
    section_4_l = 12.5
    section_5_l = 12.0
    section_5_sw = 0.5*min(3.0, 1.3*w + 0.25)  # semi-width of lane
    lane_offset = 1.0

    section_3_x_min = section_1_l + section_2_l
    section_4_x_min = section_1_l + section_2_l + section_3_l
    section_5_x_min = section_1_l + section_2_l + section_3_l + section_4_l
    total_l = section_1_l + section_2_l + section_3_l + section_4_l + section_5_l
    y_lb_12 = -section_1_sw
    y_lb_45 = -section_5_sw

    polyhedra = []

    taper = 0.25

    # lateral position in sections 1-2 is lower bounded by y >= y_lb_12
    polyhedra.append(Polyhedron([Vertex(x=0, y=y_lb_12),
                                 Vertex(x=section_3_x_min, y=y_lb_12),
                                 Vertex(x=section_3_x_min+section_3_l,y=min(y_lb_12, y_lb_45))], passing_direction=1))
    # lateral position in sections 4-5 is lower bounded by y >= -y_lb_45
    polyhedra.append(Polyhedron([Vertex(x=section_3_x_min, y=min(y_lb_12, y_lb_45)),
                                 Vertex(x=section_4_x_min, y=y_lb_45),
                                 Vertex(x=total_l, y=y_lb_45)], passing_direction=1))

    # lateral position is upper bound by y <= section_1_sw + lane_offset + 2*section_5_sw
    y_ub = section_1_sw + lane_offset + 2*section_3_sw
    polyhedra.append(Polyhedron([Vertex(x=0, y=y_ub),
                                 Vertex(x=total_l, y=y_ub)], passing_direction=-1))

    # obstacle modeling the left (in forward driving direction) lane boundary in section 1
    polyhedra.append(Polyhedron([Vertex(x=0, y=section_1_sw),
                                 Vertex(x=section_1_l, y=section_1_sw),
                                 Vertex(x=section_1_l+taper, y=y_ub)], passing_direction=-1))

    # obstacle modeling the right lane boundary in section 3
    section_3_y_min = section_1_sw + lane_offset
    polyhedra.append(Polyhedron([Vertex(x=section_3_x_min-taper, y=y_lb_12),
                                 Vertex(x=section_3_x_min, y=section_3_y_min),
                                 Vertex(x=section_4_x_min, y=section_3_y_min),
                                 Vertex(x=section_4_x_min+taper, y=y_lb_45)], passing_direction=1))

    # obstacle modeling the left lane boundary in section 5
    polyhedra.append(Polyhedron([Vertex(x=section_5_x_min-taper, y=y_ub),
                                 Vertex(x=section_5_x_min, y=section_5_sw),
                                 Vertex(x=section_5_x_min+section_5_l, y=section_5_sw)], passing_direction=-1))

    # obstacles valid at all time steps
    return dict.fromkeys(range(0, N+1), polyhedra)
