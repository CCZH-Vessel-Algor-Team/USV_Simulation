"""Local north approach geometry and finished terrain grade, in CCS ENU."""
import math


JOINT_Y = 894.0
END_Y = 1025.0
SUPPORT_Y = (839.0, 868.0)
GRADE_START_Y = 867.0
GRADE_END_Y = 1050.0


def smoothstep(a, b, x):
    t = min(1.0, max(0.0, (x-a)/(b-a)))
    return t*t*(3-2*t)


def _hermite(y, knots):
    for (a, p, da), (b, q, db) in zip(knots, knots[1:]):
        if y <= b:
            length = b-a
            t = max(0.0, (y-a)/length)
            d = q-p
            c, e = 3*d-length*(2*da+db), -2*d+length*(da+db)
            return p+t*(length*da+t*(c+t*e)), (length*da+t*(2*c+3*t*e))/length
    b, q, db = knots[-1]
    return q+db*(y-b), db


def section(y, first):
    """Continue the bridge's initial position, tangent, elevation and width."""
    x0, y0, (nx, ny), width, _, z = first
    # Follow the chart towards the island, then curve toward the gentler
    # inland shoulder. The inland termination is a local scene approximation.
    x, dx = _hermite(y, ((y0, x0, -ny/nx), (875, -631.4, .53),
                         (920, -607, .59), (958, -583, .72),
                         (982, -561, 1.16), (END_Y, -495, 1.57)))
    top, _ = _hermite(y, ((y0, z, 0), (880, z+2.2, .082),
                          (920, z+5.8, .101), (970, z+10.8, .091),
                          (END_Y, z+14.8, .022)))
    width -= 8*smoothstep(912, 980, y)
    width -= 4*smoothstep(1000, END_Y, y)
    normal = (1/math.hypot(1, dx), -dx/math.hypot(1, dx))
    bottom = top-(2.2-1.52*smoothstep(JOINT_Y, JOINT_Y+12, y))
    return x, y, normal, width, bottom, top


def sections(first):
    """Road sections in bridge station order, from inland to the old join."""
    ys = {first[1], JOINT_Y, END_Y, *SUPPORT_Y}
    ys.update(range(math.ceil(first[1]/4)*4, int(END_Y), 4))
    rows = [(0.0, first)]
    for y in sorted(ys):
        if y <= first[1]:
            continue
        row = section(y, first)
        station, previous = rows[-1]
        rows.append((station-math.dist(row[:2], previous[:2]), row))
    return rows[::-1]


def channel(y, x, first):
    """Nearest transverse road coordinates, including behind the inland end."""
    # Orthogonal projection onto the smooth alignment by successive tangent
    # projections; the local grading domain stays well inside the curve radius.
    q = min(GRADE_END_Y, max(first[1], y))
    for _ in range(8):
        cx, _, (nx, ny), _, _, _ = section(q, first)
        dy = ((x-cx)*(-ny)+(y-q)*nx)*nx
        q = min(GRADE_END_Y, max(first[1], q+dy))
    cx, _, (nx, ny), width, _, top = section(q, first)
    offset = (x-cx)*nx+(y-q)*ny
    return q, offset, width, top


def grade(x, y, z, first):
    """Regrade only the existing island surface, tapering to the old slopes."""
    if not (-730 < x < -395 and 838 < y < 1080) or z < .26:
        return z
    q, offset, width, top = channel(y, x, first)
    edge = abs(offset)-width/2
    weight = smoothstep(GRADE_START_Y, JOINT_Y, q)
    weight *= 1-smoothstep(4, 38, edge)
    weight *= (1-smoothstep(END_Y, GRADE_END_Y, q))*smoothstep(.25, 2.5, z)
    # A flat roadbed plus forgiving cut/fill shoulders, not a vertical box.
    target = top-.28-.38*max(0.0, edge-1.8)
    return z+(target-z)*weight
