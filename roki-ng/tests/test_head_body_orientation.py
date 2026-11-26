import math
import pytest
from roki_ng.body.head_body_orientation import multiply, inverse, rotation, body_from_head


def test_known_neck_rotation_is_removed_in_correct_order():
    body=rotation((0,0,1),.7)
    mount=rotation((1,0,0),.3)
    neck=multiply(rotation((0,1,0),.4),rotation((1,0,0),-.6))
    alignment=rotation((0,0,1),-.8)
    head=multiply(inverse(alignment),multiply(body,multiply(neck,mount)))
    predicted=body_from_head(head,.4,-.6,mount,alignment)
    assert abs(sum(a*b for a,b in zip(body,predicted)))==pytest.approx(1)


def test_zero_or_nonfinite_quaternion_is_rejected():
    for q in ((0,0,0,0),(0,0,float('nan'),1)):
        with pytest.raises(ValueError):body_from_head(q,0,0,(0,0,0,1),(0,0,0,1))
