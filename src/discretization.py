import numpy as np
import networkx as nx
from utils import normalize_angle

class GraphBuilder:
    def __init__(self, world, v: float, tau: float):
        self.world = world
        self.L = v * tau
        self.tau = tau
        # headings 0..7 represent multiples of 45°
        self.headings = list(range(8))

    def build(self, xmin=-1, xmax=1, ymin=-1, ymax=1):
        G = nx.DiGraph()
        # generate lattice points
        xs = np.arange(xmin, xmax + 1e-6, self.L)
        ys = np.arange(ymin, ymax + 1e-6, self.L)
        nodes = []
        for x in xs:
            for y in ys:
                if self.world.point_in_free(x, y):
                    for h in self.headings:
                        nodes.append((round(x,3), round(y,3), h))
        # add nodes
        G.add_nodes_from(nodes)
        # add edges
        for x, y, h in nodes:
            # turn edges: +1 and -1 heading mod 8
            for dh in (-1, 1):
                h2 = (h + dh) % 8
                G.add_edge((x,y,h), (x,y,h2))
            # move edges with micro-turns <=15° (1 heading unit)
            for dh in (-1, 0, 1):
                h2 = (h + dh) % 8
                angle = h2 * (np.pi/4)
                x2 = x + self.L * np.cos(angle)
                y2 = y + self.L * np.sin(angle)
                # line-of-sight check
                if self.world.segment_in_free((x,y), (x2,y2)) and self.world.point_in_free(x2,y2):
                    G.add_edge((x,y,h), (round(x2,3), round(y2,3), h2))
        return G