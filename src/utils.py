import math

def bearing(p, q):
    return math.atan2(q[1]-p[1], q[0]-p[0])

# Visibility and detection logic

def normalize_angle(angle):
    """Normalize angle to [-pi, pi)."""
    return (angle + math.pi) % (2*math.pi) - math.pi


def is_visible(pos_o, head_o, p, world, r, gamma):
    """Return True if point p is visible from observer at pos_o with 
heading head_o."""
    # ensure both points in free space
    if not world.point_in_free(pos_o[0], pos_o[1]) or not 
world.point_in_free(p[0], p[1]):
        return False
    # distance check
    dx, dy = p[0] - pos_o[0], p[1] - pos_o[1]
    dist = math.hypot(dx, dy)
    if dist > r:
        return False
    # field-of-view check
    angle = math.atan2(dy, dx)
    if abs(normalize_angle(angle - head_o)) > gamma:
        return False
    # line-of-sight check
    if not world.segment_in_free(pos_o, p):
        return False
    return True


def is_detected(pos_t, head_t, pos_o, world, r_d, r, gamma):
    """Return True if observer at pos_o is detected by target at pos_t."""
    dx, dy = pos_o[0] - pos_t[0], pos_o[1] - pos_t[1]
    dist = math.hypot(dx, dy)
    # omnidirectional detection
    if dist <= r_d:
        return True
    # directional detection within view
    if dist <= r:
        bearing = math.atan2(pos_o[1] - pos_t[1], pos_o[0] - pos_t[0])
        if abs(normalize_angle(bearing - head_t)) <= gamma and 
world.segment_in_free(pos_t, pos_o):
            return True
    return False
