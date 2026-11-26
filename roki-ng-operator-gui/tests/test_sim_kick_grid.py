from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from sim_kick_grid import target


def test_recommendation_requires_successes_around_target_and_no_failed_repeats():
    trials = [dict(placement=[x, y], usable=True) for x in (125,130,135) for y in (47,52,57)]
    assert target(trials, [150,52]) == dict(front_mm=130, side_mm=52, spacing_mm=5)
    assert target(trials[:-1], [150,52]) is None
    trials.append(dict(placement=[135,52], usable=False))
    assert target(trials, [150,52]) is None
