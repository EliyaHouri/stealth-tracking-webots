# src/world_generator.py
def generate_obstacles(seed, n, boundary):
    # ignore seed/n; place one central rectangle half the world size
    minx, miny = boundary[0]
    maxx, maxy = boundary[2]
    cx, cy = (minx+maxx)/2, (miny+maxy)/2
    w, h = (maxx-minx)/5, (maxy-miny)/5
    rect = [
        (cx - w/2, cy - h/2),
        (cx + w/2, cy - h/2),
        (cx + w/2, cy + h/2),
        (cx - w/2, cy + h/2)
    ]
    return [rect]
