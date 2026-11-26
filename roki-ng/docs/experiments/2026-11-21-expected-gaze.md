# Simple football: expected gaze and aiming, 2026-11-21

## Implemented

- Store short-lived **gaze hints**, independent of observations authorizing kicks.
- For a confirmed enemy goal opening choose a specific image-side post. Preserve its index when the opening is seen again. An isolated colour blob cannot establish post identity or overwrite it.
- Compensate target bearing with body IMU yaw. Respect the robot model's camera pan sign. Reject stale IMU, non-upright pose, changed yaw epoch and out-of-head-range bearings.
- Ball hints expire after 6 seconds, post hints after 15 seconds. These are heading hints, not translation-corrected world positions. Fresh detection is still required for movement/strike decisions.
- Inspect the predicted post first, then a nearby direction; the wider sweep is a fallback. Return to the predicted ball direction, with a centre fallback if the ball is missing.
- Reacquiring a close ball with the head no longer automatically turns the body and undoes goal alignment.
- If the ball is aligned with the foot but the goal is only a colour hint, hold the feet and observe again. Remove speculative sideways orbit/face-ball cycles.
- A shot now also requires at most 3 degrees error to the chosen free lane centre. Fine turns are bounded to 10 degrees and require a fresh lane. Existing own-goal, occupancy and freshness checks remain.

Robot-specific settings saved through params.set:

- `walk.body_tilt_forward = 0.065` (unchanged)
- `game.simple_football.near_head_tilt = -2100` (previously -1800)

## Evidence and limitations

`output/football-check-20261121-180822/` contains the live recording and IMU/status log. At 55.29 s the strategy requested a right-foot kick after 12 alignment jumps. The body reported completion; video shows the ball moving forward, right of the goal mouth. This is **a verified ball strike, not a verified goal**. The 3-degree precision gate and removal of speculative orbiting were added afterwards and have not yet been verified by another live strike.

IMU pitch during alignment was -2.0..8.1 degrees and during the strike -7.9..4.5 degrees. The subsequent long approach fell at about 69.6 s. A separate repeat (`181042`) also fell on the long approach. Both bounded tests stopped. Therefore neither reliable far approach nor end-to-end autonomous scoring is established.

A yellow background detection sometimes activates the own-goal veto. This remains unresolved; do not disable the veto to make a shot pass.

Live logs confirm expected-post gaze after body turns. The -2100 ball view keeps the close ball visible in the tested poses. Full single-post identity inference without a previously observed opening is not implemented.

## Verification

Run from project root:

```sh
python -m pytest tests/test_expected_gaze.py tests/test_football_vision.py tests/test_legacy_game_motion.py -q
```

37 passed, 7 skipped locally. This is a focused suite, not a claim that the entire repository suite passes. Existing approach tests contain older strategy expectations and require separate review.

Next live test: ball 20–25 cm in front of upright robot, blue goal ahead; verify no incidental ball contact during adjustment and stop after the first kick. Address the repeatable forward fall before resuming long approaches.

## Subsequent live checks

- `181843`: close approach reproduced repeated own-goal veto. Replaying the image showed the yellow mask on the green/white perimeter fringe, not a yellow goal.
- Added rejection when more than 65% of a colour component lies in the immediate white-marking neighbourhood. On 60 sampled frames from this recording, blue-opening frames remained 12/12; yellow detections decreased from 37 to 3. Subsequent live views still produced occasional yellow false positives; filtering is not complete.
- `182305`: another bounded close alignment. A strike was requested at estimated 2.6-degree error but coincided with operator stop; completion was not confirmed. Do not count this as a successful shot.
- Removed the four-second goal-scan timer in close alignment and the premature return when an old goal opening lacks a fresh free lane. Now an initial requested scan is followed by ball alignment, and a new scan is requested when foot alignment is ready but the shot is not confirmed.
- Focused suite after these changes: 39 passed, 7 skipped.
- `182519` / `182614`: ball estimated at 0.53–0.55 m. The test harness stopped before long approach, as intended.
- A stationary head view at tilt -2400 showed both toe tips; head restored to -2100. No automatic toe detector or foot-contact guard has been implemented yet. One manual half-amplitude forward jump completed upright, with little visible displacement.
- `simple_football` still forces visual localisation off. Approximate field pose integration remains outstanding; do not substitute the default odometric starting position for a measured field pose.

## Stationary tilt sweep (18:32 Moscow)

User measured 0.50 m from robot centre and 0.45 m from toe tips to ball centre. Robot remained stationary during `output/camera-tilt-sweep-20261121-183255/`:

| Head tilt | Median forward ball distance | Unique samples |
|---|---:|---:|
| -1000 | 0.5117 m | 13 |
| -1500 | 0.5091 m | 17 |
| -1800 | 0.5126 m | 17 |
| -2100 | 0.5127 m | 17 |
| -2400 | 0.4889 m | 18 |
| -1500 repeat | 0.5094 m | 17 |

At -1000..-2100 the spread is 3.6 mm and absolute disagreement with the hand measurement about 9–13 mm. No global scale correction applied. The extreme -2400 view shows toe tips but shifts the ball estimate; do not use it as the sole ranging view. User-measured toe offset is +0.05 m relative to the robot centre, so toe-to-ball-centre forward gap is `x_m - 0.05`. This is distinct from surface clearance, which additionally subtracts measured ball radius. Existing 100-mm kick target is robot-centred, equivalent to 50-mm toe-to-ball-centre gap. No visual foot detector is claimed.
