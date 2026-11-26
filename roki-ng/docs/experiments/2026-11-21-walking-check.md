# Walking checks, 21 November 2026 (Moscow)

## Scope and findings

Tests used manual `motion.drive`, speed 0.35, heading hold, fresh body IMU and camera recording. Each run ended with zero drive and return to base stance. A 14-degree pitch/roll excursion requested early stopping. No autonomous get-up or football continuation was requested by this test harness.

The tests establish bounded manual walking without detected falls, not stable autonomous football approach or a measured one-metre walk. Automatic approach previously fell; it remains unverified with these settings. A person entered the forward path in the last recording, so further movement was suspended.

## Changes

`Body._pose('base_stand')` now respects `walk.crouch_transition_frames` when leaving crouch. Previously the Initial_Pose slot always used its own 20-frame timing. Only this targeted runtime change was deployed; the separate local adaptive-stride draft was not deployed.

Saved robot settings:

- `walk.body_tilt_forward = 0.065` (unchanged)
- `walk.crouch_transition_frames = 40` (was 20)
- `walk.sway_amplitude_mm = 16` (was 24)
- `walk.step_height_mm = 24` (trial at 16 reverted after larger roll excursion)
- `walk.max_step_mm = 24` (unchanged)

The existing gait keeps elbows bent and has a small shoulder term based on opposite-foot displacement. This is not an IMU-driven arm balance controller; no such controller was added.

## Recorded results

All ranges are IMU degrees. Single trials are not statistically controlled comparisons.

| Recording suffix | Test | Pitch range | Roll range |
|---|---|---|---|
| 183951 | Initial short-test stop | -9.82..7.68 | -9.69..10.09 |
| 184155 | 40-frame transition, stop | -6.65..9.20 | -10.32..5.50 |
| 184242 | sway 16, step height 24, stop | -7.26..6.66 | -8.64..8.02 |
| 184310 | step height 16 trial, stop | -6.10..6.37 | -10.38..13.21 |
| 184341 | 3-second drive, walking | -6.94..6.10 | -11.26..9.34 |
| 184448 | 5-second drive, walking | -7.16..6.44 | -10.30..10.54 |
| 184639 | 10-second drive, walking | -7.44..5.20 | -12.19..12.29 |

Files are `output/imu-recovered-walk-20261121-<suffix>.jsonl` and `.mkv` in the workspace.

The last run reported 14 completed gait cycles, ended at base_stand with no active job, and showed about -15.9 degrees net yaw change. The configured walking yield predicts about 18 cm for these cycles, but this is not an independent measurement. One metre has not been verified.

A separate positive manual yaw probe (`184534`) gave about -6.46 degrees measured yaw. Therefore feedback signs must be checked against physical motion rather than changed from a visual guess. No yaw feedback sign was changed in this session.

## Verification and remaining work

16 body-connection tests passed including a regression that crouch exit uses configured interpolation duration. Focused gaze/vision/legacy motion suite: 40 passed, 7 skipped.

Remaining: independently measure translation and course, correlate roll with support phase, validate a smoother terminal cycle, then evaluate bounded arm corrections in isolation. Do not increase arm feedback gains or claim adaptive balance from the current results.

## Meter-length command after user repositioning

User explicitly confirmed the path clear and repositioned the robot. Distance accounting restarted at recording `184918`; earlier runs were excluded. Subsequent completed manual-drive jobs:

| Recording | Completed cycles | Peak absolute pitch | Peak absolute roll |
|---|---:|---:|---:|
| 184918 | 29 | 8.58 deg | 13.31 deg |
| 185003 | 29 | 8.69 deg | 12.68 deg |
| 185045 | 19 | 12.79 deg | 11.05 deg |

All three ended at `base_stand`, no active job, and without the 14-degree guard triggering. Video shows forward travel toward the blue goal. The final segment requested stopping after progress reached 18; one additional already-dispatched cycle completed, yielding 77 cycles total.

Using the configured yield (100 mm per cycle at 64-mm command stride), max-step 24 mm and speed 0.35 gives `77 * 100 * (24 * 0.35 / 64) = 1010.625 mm`. This is a **calibration-derived path estimate**, not an independent ruler measurement or proof of a straight one-metre displacement. Robot stopped before the goal structure. No fall was observed in these three runs. Roll and heading drift still require improvement; do not label the gait production-stable from this trial.
