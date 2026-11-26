import numpy as np
import pytest

from roki_ng.detection.ball_boundary import outside_paint
from roki_ng.localisation.field_config import GEOMETRY


@pytest.mark.parametrize('stripe,edge,out', [(40,56,True), (48,64,False), (56,72,False), (40,96,False)])
def test_painted_edge_and_halfway_line(stripe, edge, out):
    white = np.zeros((97,97), np.uint8)
    green = np.zeros_like(white)
    green[:,:edge] = 255
    white[:,stripe-1:stripe+2] = 255
    green[white != 0] = 0
    valid = np.ones_like(white, bool)
    for _ in range(4):
        assert outside_paint(green,white,valid,GEOMETRY) == out
        green,white = np.rot90(green),np.rot90(white)
    valid[:,edge:] = False
    assert not outside_paint(green,white,valid,GEOMETRY)
