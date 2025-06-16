import numpy as np
import networkx as nx
from world_model import WorldModel
from utils import discretize_heading

class GraphBuilder:
    def __init__(self, world: WorldModel, v: float, tau: float):
        self.world = world
        self.L = v * tau
        self.tau = tau
        self.Theta8 = [k*np.pi/4 for k in range(8)]

    def build(self):
        G = nx.DiGraph()
        # generate lattice, nodes, edges lazily (omitted for brevity)
        return G
