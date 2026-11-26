from types import SimpleNamespace
import pytest
from roki_ng.worker import poll_timeout


def test_motion_deadline_wakes_worker_before_idle_poll():
    body = SimpleNamespace(plan=object(), body_connected=True,
                           motherboard_reset_pending=False, queue_reset_pending=False, next_at=10.003)
    assert poll_timeout('motherboard', body, 10) == pytest.approx(.003)
    assert poll_timeout('motherboard', body, 10.005) == .001
    body.plan = None
    assert poll_timeout('motherboard', body, 10) == .01
    assert poll_timeout('camera', object(), 10) == .01
