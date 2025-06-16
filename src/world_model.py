import shapely.geometry as geom
import json

class WorldModel:
    def __init__(self, obstacle_file: str):
        data = json.load(open(obstacle_file))
        self.workspace = geom.Polygon(data['boundary'])
        self.obstacles = [geom.Polygon(o) for o in data['obstacles']]

    def point_in_free(self, x: float, y: float) -> bool:
        p = geom.Point(x, y)
        if not self.workspace.contains(p):
            return False
        for o in self.obstacles:
            if o.contains(p):
                return False
        return True

    def segment_in_free(self, a, b) -> bool:
        seg = geom.LineString([a, b])
        if not self.workspace.contains(seg):
            return False
        for o in self.obstacles:
            if seg.intersects(o):
                return False
        return True
