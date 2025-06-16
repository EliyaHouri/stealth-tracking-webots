import pytest
from src.world_model import WorldModel

def test_point_and_segment():
    w = WorldModel('obstacles.json')
    assert w.point_in_free(0, 0)
