# Long run: forward fall and shared feedback

## Evidence

Source: `output/long-run-calibration-20261121-190002/samples.jsonl`.
The operator confirmed a forward fall. Completed-cycle counters below are not
foot-contact phase measurements; polling cannot identify exact touchdown.

| Completed cycles | Time interval, s | Pitch range, deg | Roll range, deg |
|---|---|---|---|
| 0 | 0.02–1.63 | -8.18…6.08 | -0.92…5.41 |
| 1 | 1.83–2.35 | 3.87…14.54 | -8.91…7.18 |
| 2 | 2.46–3.10 | -8.38…-1.08 | -4.47…4.05 |
| 3 | 3.21–3.73 | 0.26…9.40 | -7.91…19.86 |
| 4 | 3.92–4.45 | -2.38…12.19 | -9.87…16.69 |
| 5 | 4.57–4.98 | 12.19…41.69 | -2.69…18.45 |

Pitch diverged after the fifth completed cycle; roll was already large earlier.
The calibration runner cancelled during the fall transition. No distance saved.

## Confirmed code discrepancy and fix

Runtime enabled per-servo IMU feedback for `motion.drive` and `game.walk`, but
excluded `test.start`. Short/Long run used the same gait engine without its live
pitch feedback. This is a confirmed difference, not proof it is the only cause
of falling. Long/Short runs ramp to 64 mm; successful manual trials used much
smaller commands.

A scoped `calibration_walking` flag now selects the SAME runtime regulator while
TestPlan.gait runs. It is cleared on generator exit and job finish. Recovery,
kicks, jumps and other test segments do not opt into walking feedback.
No second regulator or separately tuned Long-run gait was introduced.

Base `walk.body_tilt_forward` stays 0.065. Existing feedback settings are reused:
proportional gain 0.2, correction bounded to 2 degrees, slew 1 degree/second.
The correction acts on hip targets; it does not persist a new base tilt or change
stride. This is bounded feedback, NOT a proven automatic parameter optimizer.
The prior experimental adaptive-stride draft remains undeployed.

## Verification and remaining work

71 focused tests passed, 8 skipped. 9 supervisor/network cases excluded from
this focused run; tested supervisor cases fail on macOS due to missing
CLOCK_BOOTTIME, unrelated to this patch. New regression covers feedback during
calibration, freezing outside gait, and Short/Long flag cleanup.

Only the scoped runtime patch and calibration.py were copied to the robot,
preserving other local undeployed drafts. Live repeat is pending upright robot
and a clear route. Runners now record body.stabilization alongside IMU and catch
posture classification faults rather than terminating with a traceback.

Do not claim stable Long run until a full live repeat succeeds. If controller
behavior changes, repeat BOTH Short and Long physical distance measurements;
the earlier 950 mm Short result belongs to the previous controller behavior.

## Live repeat after full power cycle

`output/long-run-calibration-20261121-193535/`: IMU quaternion varied before
launch with pitch about -2.2 degrees. Long run completed 5 cycles before graceful
cancellation. At 4.08 s, roll/pitch reached 30.5/19.8 degrees; at 4.23 s,
27.8/37.0 degrees. A hand is visible supporting the robot in the final video
frame; do not treat its return upright as autonomous recovery.

Feedback was applied (offset -0.776 degrees), but the existing regulator froze
updates with `tilt_outside_range` during the large excursion. It resumed below
its operating range. This patch alone has NOT established stable Long run.
No calibration measurement was saved. Base tilt remains 0.065.
