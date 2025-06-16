import pytest
from src.discretization import GraphBuilder
from src.world_model import WorldModel

def test_graph_small():
    wm = WorldModel('obstacles.json')
    gb = GraphBuilder(wm, v=0.1, tau=1)
    G = gb.build()
    assert isinstance(G, nx.DiGraph)
