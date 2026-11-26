"""Conservative offline scoring for measured gait trials, not an IMU controller."""
import math
import statistics


def candidates(base):
    return [round(base + d, 6) for d in (-.001, .001) if -.3 <= base + d <= .3]


def trial_score(samples, *, completed=True):
    if not completed or len(samples) < 20:
        return math.inf
    if any(not math.isfinite(v) for row in samples for v in row):
        return math.inf
    pitch, roll = zip(*samples)
    # Penalize persistent lean AND alternating sway; averaging alone hides sway.
    rms = math.sqrt(statistics.mean(p*p + r*r for p, r in samples))
    peak = max(math.hypot(p, r) for p, r in samples)
    return rms + .25*peak + .5*abs(statistics.mean(pitch))


def choose(base, baseline_score, trials):
    if not math.isfinite(baseline_score):
        return base
    approved = [(max(scores), value) for value, scores in trials
                if len(scores) >= 2 and all(math.isfinite(s) for s in scores)
                and max(scores) < .9*baseline_score]
    return min(approved)[1] if approved else base


async def search(base, measure, *, max_trials=10):
    """Repeated local search; measure must raise for falls or missing telemetry.

    No moves beyond three thousandths of the starting value in one session.
    Cached repeats avoid deliberately revisiting known inferior candidates.
    The returned candidate still needs full-stride validation.
    """
    if not math.isfinite(base) or not -.3 <= base <= .3 or max_trials < 2:
        raise ValueError('Invalid search baseline or budget')
    measured = {}
    trials = 0
    async def pair(value):
        nonlocal trials
        scores = []
        for _ in range(2):
            score = await measure(value)
            trials += 1
            if not math.isfinite(score):
                raise RuntimeError('Unusable gait trial; search aborted')
            scores.append(score)
        measured[value] = scores
    await pair(base)
    best = base
    while True:
        neighbors = [v for v in candidates(best) if abs(v-base) <= .00300001]
        for value in neighbors:
            if value not in measured and trials+2 <= max_trials:
                await pair(value)
        winner = choose(best, min(measured[best]),
                        [(v, measured[v]) for v in neighbors if v in measured])
        if winner == best:
            break
        best = winner
        if trials >= max_trials:
            break
    return {'candidate': best, 'baseline': base, 'trial_count': trials,
            'scores': [{'value': v, 'repeats': scores} for v, scores in measured.items()],
            'validated_full_stride': False}
