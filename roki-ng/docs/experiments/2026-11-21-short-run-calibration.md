# Short run calibration — 21 November 2026

Requested calibration uses the existing `test.start` API with `name=run_test`, `mode=short`, not a timed manual drive. The implementation performs 11 cycles, ramps the first two toward the standard 64-mm stride, and identifies `motion.run_10_mm` as the measured parameter.

Live artifact directory: `output/short-run-calibration-20261121-185319/`.

- Job completed all 11 cycles with no failure reason, final reported pose `stand`.
- Recorded pitch range -15.69..6.76 degrees, roll -9.26..10.67 degrees; the most negative pitch appears at completion, after the running-job guard.
- Net IMU yaw change approximately -13.67 degrees. Record lateral displacement separately; do not assume a perfectly straight trial.
- Video and full sampled telemetry retained.
- Manual distance measurement requested; no new coefficient recorded yet.

The earlier user-reported 130 cm was not applied to this test. It came from the preceding manual-drive sequence and its measurement scope was not confirmed before the user selected Short run.

The odometry uses both short and long distances: subsequent-cycle yield `(long-short)/10`, first-cycle yield `(19*short-9*long)/10`. A complete calibration should include Long run and a verification pass; one Short run must not be presented as fully calibrated odometry.

## Saved measurement

User measured **95 cm** for this Short run. Saved through the same live job's `test.measure` request: `motion.run_10_mm = 950.0`. Robot API confirmed the save. Lateral displacement was not supplied.

Long-run coefficient remains the previous 2000 mm and has not been calibrated in this trial. Together these values imply first-cycle yield 5 mm and subsequent-cycle yield 105 mm at reference stride; this is an unverified combination. Perform Long run and validation before trusting full odometric distances. No automatic proportional change was made to the long-run value.

## Long run attempt

`output/long-run-calibration-20261121-185653/`: requested stock Long run (21 cycles). On the first completed cycle, fresh body IMU reported roll -14.30 degrees, crossing the harness's 14-degree guard. Graceful cancellation completed after cycle 2; job status `cancelled`, reason `graceful_stop`. This is not a valid calibration trial. No `test.measure` was submitted; `motion.run_20_mm` remains 2000 mm. Do not increase the threshold just to obtain a completed calibration.

## Correction: diagnostic cancellation

The operator rejected cancelling for ordinary gait sway. Removed the external
14-degree cancellation condition from both local calibration runners. Fresh IMU
and existing body-position fall checks remain. This changes the test harness,
not the robot gait controller. No rerun or new calibration measurement yet.
Adaptive stride is a separate, undeployed experiment: changing stride during
stock Short/Long run would invalidate the fixed-stride odometry calibration.
