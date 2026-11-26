import math
import pytest
from roki_ng.body.walk_tuning import trial_score, candidates, choose


def samples(pitch=2, roll=2):
    return [(pitch, roll)] * 30


def test_fall_and_missing_data_cannot_win():
    assert math.isinf(trial_score(samples(0, 0), completed=False))
    assert math.isinf(trial_score([(0, 0)]))
    assert math.isinf(trial_score(samples() + [(float('nan'), 0)]))


def test_large_sway_is_worse_than_small_constant_lean():
    assert trial_score([(12*(-1)**i, 10*(-1)**i) for i in range(30)]) > trial_score(samples(3, 2))


def test_small_change_and_repeated_improvement_required():
    assert candidates(.065) == pytest.approx([.064, .066])
    assert choose(.065, 10, [(.064, [7, 11]), (.066, [8, 8.5])]) == .066
    assert choose(.065, 10, [(.064, [9.8, 9.9])]) == .065


def test_search_continues_in_better_direction_and_respects_trial_budget():
    import asyncio
    from roki_ng.body.walk_tuning import search
    seen = []
    async def measure(value):
        seen.append(value)
        return 2 + 4000*abs(value-.063)
    result = asyncio.run(search(.065, measure, max_trials=10))
    assert result['candidate'] == pytest.approx(.063)
    assert len(seen) <= 10
    assert abs(result['candidate']-.065) <= .003
    assert seen.count(.063) >= 2


def test_failed_trial_aborts_search_without_trying_other_values():
    import asyncio
    from roki_ng.body.walk_tuning import search
    seen = []
    async def measure(value):
        seen.append(value)
        raise RuntimeError('fall')
    with pytest.raises(RuntimeError, match='fall'):
        asyncio.run(search(.065, measure))
    assert seen == [.065]


def test_noise_cannot_move_baseline():
    import asyncio
    from roki_ng.body.walk_tuning import search
    async def measure(value):
        return 10 if value == .065 else 9.8
    result = asyncio.run(search(.065, measure))
    assert result['candidate'] == .065
