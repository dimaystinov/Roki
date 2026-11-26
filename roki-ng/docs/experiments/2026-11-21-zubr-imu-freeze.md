# Zubr body IMU freeze after Long run

The operator confirmed the robot was upright, while body IMU continued reporting
approximately roll 9.6 degrees and pitch 83.6 degrees. Walking was not started.

## Isolating the source

Stopped `roki-head` before opening the motherboard transport directly. Used a
new Python process and successful `Rcb4.checkAcknowledge()`, then read documented
Kondo RAM registers:

- 0x60, 8 bytes: quaternion, repeatedly `[427, -16351, -913, 179]`.
- 0xc0, 12 bytes: accelerometer and gyro, repeatedly
  `[78, 16331, 192, -5, -11, 9]`.
- 0xcc, 2 bytes: supply ADC changed (approximately 3350–3361).

Thus the transport and controller respond to fresh requests; the stale values
are already in Zubr's sensor data, not introduced by the operator GUI or
roki-ng cache. Head motherboard `imu_healthy` describes the head IMU and does
not certify the body sensor. `BodyImu.sequence` counts successful host reads;
its host-receive timestamp does not prove a new measurement occurred.

After the operator's first reported power cycle, all ten IMU values remained
bit-identical. True loss of power to the body sensor has not been verified;
possible alternate supply needs checking. Service was restarted without issuing
motion commands. The sensor has NOT been recovered yet.

## Firmware evidence (not confirmed installed revision)

Local reference:
`/Users/mac/Documents/ChatGPT/Roki_reverse/references/user-controller/roki-zubr-firmware/source/HwImu.cpp`.

This driver initializes BHI260 and then polls interrupt status and drains the
FIFO. Its steady-state loop ignores return values from both operations and
contains no sample-age watchdog or reinitialization. The parsed quaternion has
a sensor timestamp, but the inspected Kondo quaternion read exports only the
components. This is a plausible recovery gap, NOT proof of the precise
hardware/firmware fault or installed version.

Required durable fix should be made against the verified controller firmware:
check communication errors and sensor sample timestamps, invalidate stalled
samples, perform bounded sensor recovery, expose sensor age/health, and do not
silently resume movement after recovery. Do not use changes of quantized
quaternion alone as proof of health: a stationary sensor can repeat values.
No guessed register writes, firmware flashing or automatic get-up were attempted.

Long run remains pending verified recovery and upright body IMU. Base tilt is
still 0.065; shared Short/Long/football pitch feedback is installed independently.

## Recovery outcome

After the second operator-confirmed full power interruption, host uptime reset
and quaternion components changed across successive samples. Pitch was about
-2.2 degrees and roll about 0.3 degrees, matching upright stance. This restored
live operation by power cycle; it does not fix the firmware recovery gap.
A subsequent Long run was recorded separately in the feedback report.
