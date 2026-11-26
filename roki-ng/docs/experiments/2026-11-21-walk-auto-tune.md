# Bounded automatic walking tilt comparison

Implemented in `tools/tune_walk_tilt.py` and `roki_ng/body/walk_tuning.py`.
The existing `walk.body_tilt_forward` key is used; there is no separate football,
Short-run or Long-run tilt. The tuner does not replace per-frame IMU feedback.

## Search

- Two baseline trials, then two trials per neighboring tilt at increments 0.001.
- A neighbor must improve both repeats by at least 10% versus the better baseline
  repeat. A single fortunate run cannot win.
- Continue around the winner until no improvement, 10 trials, or ±0.003 from the
  session baseline. This is a bounded local search, not a global optimum claim.
- Identical three-cycle commands, step 24 mm, same startup ramp and settings.
- Score pitch/roll RMS, peak combined excursion and mean pitch bias; include
  settling. A fall, failed job or unreliable telemetry aborts the session.
- Store a candidate in the journal; restore original parameters and verify
  readback. Candidate requires separate Short/Long full-stride validation before
  adoption. Changing the controller requires repeating distance calibration.
- Parameter API persists trial values temporarily. Rollback is best effort if
  connection is lost; journal `restored` and `restore_error` must be checked.
- No automatic get-up or continuation after a fall.

## Actual evidence

`output/walk-auto-tune-20261121-193915` failed during its first baseline trial at
0.065. The operator confirmed an immediate forward fall. Further candidates
were NOT run. Baseline was restored and verified. No successful tilt found.

Later the body quaternion again repeated bit-identically while the operator
reported upright stance. Live search is blocked until source IMU is recovered;
new host read sequence numbers do not prove fresh sensor measurements.

Six unit tests cover failed/missing data, sway scoring, repeated improvement,
continuing toward a better candidate, budget and failure propagation.
Physical validation of the sequential search remains pending.
