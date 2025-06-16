import json
from shapely.geometry import Point, Polygon, LineString

class WorldModel:
    def __init__(self, obstacle_file: str):
        data = json.load(open(obstacle_file))
        # load workspace boundary and obstacles
        self.workspace = Polygon(data['boundary'])
        self.obstacles = [Polygon(o) for o in data['obstacles']]

    def point_in_free(self, x: float, y: float) -> bool:
        p = Point(x, y)
        if not self.workspace.contains(p):
            return False
        for o in self.obstacles:
            if o.contains(p):
                return False
        return True

    def segment_in_free(self, a: tuple, b: tuple) -> bool:
        seg = LineString([a, b])
        if not self.workspace.contains(seg):
            return False
        for o in self.obstacles:
            if seg.crosses(o) or seg.within(o) or seg.intersects(o):
                return False
        return True