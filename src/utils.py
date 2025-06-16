import math


def normalize_angle(angle: float) -> float:
    """
    Normalize any angle to the range [-pi, pi).
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi


def is_visible(pos_o: tuple, head_o: float, p: tuple, world, r: float, gamma: float) -> bool:
    """
    Check if point p is visible from observer at pos_o with heading head_o.
    """
    # both observer and point must be in free space
    if not (world.point_in_free(pos_o[0], pos_o[1]) and world.point_in_free(p[0], p[1])):
        return False
    # distance check
    dx = p[0] - pos_o[0]
    dy = p[1] - pos_o[1]
    dist = math.hypot(dx, dy)
    if dist > r:
        return False
    # field-of-view check
    angle_to_p = math.atan2(dy, dx)
    if abs(normalize_angle(angle_to_p - head_o)) > gamma:
        return False
    # line-of-sight check
    if not world.segment_in_free(pos_o, p):
        return False
    return True


def is_detected(pos_t: tuple, head_t: float, pos_o: tuple, world, r_d: float, r: float, gamma: float) -> bool:
    """
    Return True if observer at pos_o is detected by target at pos_t.
    """
    dx = pos_o[0] - pos_t[0]
    dy = pos_o[1] - pos_t[1]
    dist = math.hypot(dx, dy)
    # omnidirectional detection
    if dist <= r_d:
        return True
    # directional detection: within sensor range and FOV
    if dist <= r:
        bearing = math.atan2(dy, dx)
        if abs(normalize_angle(bearing - head_t)) <= gamma:
            # also ensure line-of-sight
            if world.segment_in_free(pos_t, pos_o):
                return True
    return False