# Zubr: IMU measurement progress

## Status

Firmware patch and HEX/BIN/ELF built against the locally supplied Zubr v165
STM32H743 source. **Not flashed and not hardware-validated.** This is not an
official release and not a byte-identical reproduction of the original.

Source baseline:
`/Users/mac/Documents/ChatGPT/Roki_reverse/references/user-controller/roki-zubr-firmware`.
Check `baseline-sha256.json` before applying `imu-counter.patch` with `patch -p1`.
Baseline includes its existing DMA UART transport. Do not apply a whole firmware
image to a different controller variant merely because its version number is 165.
No changes to scripts, servo calibration, gait or bootloader are in this patch.

Built artifacts for this session:
`/Users/mac/Documents/ChatGPT/Roki_2/output/zubr-v165-imu-counter`.
No Zubr bootloader or upload procedure was confirmed for the connected board;
do not substitute the HEAD motherboard flashing utility.

## Hardware diagnostic block

Read 24 bytes via existing Kondo RAM-read command at **0x0150**:

| Offset | Type | Meaning |
|---|---|---|
| 0 | 4 bytes | ASCII `IMU1` |
| 4 | uint32 LE | Quaternion measurement sequence |
| 8 | uint32 LE | Low 32 bits of sensor timestamp, 15.625 µs/tick |
| 12 | 4 × int16 LE | Quaternion x,y,z,w, divided by 16384 |
| 20 | int16 LE | Existing initialization error field; NOT a new runtime error monitor |
| 22 | uint16 LE | Bit 0: at least one quaternion measurement received |

The counter advances in the quaternion FIFO callback only when its sensor
 timestamp differs from the preceding event. Host reads, polling loops and
identical duplicate timestamps do not increment it. Identical orientations with
advancing timestamps DO increment it. Sequence wraps modulo 2^32.
The low timestamp wraps after about 18.6 hours; consumers compare progress,
not raw ordering or elapsed time across wraps.

Reply fields are captured before Kondo reply serialization. The supplied source
runs FIFO parsing and Kondo handling as cooperative tasks, with no yield in
capture/serialization. If converted to interrupt-driven or preemptive updates,
this must be replaced with proper snapshot synchronization.

Existing registers 0x60–0x68 are unchanged. The new block avoids servo positions,
servo velocities (0xd8–0x113), and DMA diagnostics at 0x17c and above.
The marker is a hardware layout identifier, not operator protocol negotiation.

## Head runtime and screen

Head adapter reads the coherent block in one transaction. `BodyImu` refreshes
measurement age only on changed sensor sequence/timestamp. Successful reads of
a stopped sequence do not reset age. At the existing 150 ms freshness limit,
data becomes invalid for motion consumers. Read failures remain distinct.

Hardware without this block retains its older read path, explicitly reporting
`sensor_sequence: null`. A heuristic suspicion is not presented as verified
sensor health. Restart head service after firmware installation to reprobe.

LCD uses a local `/run/roki-body-imu.json` status, not a second UART owner or
loopback network connection:

- `OK` green: measurement counter advances (orientation can remain identical).
- `STALLED` red: UART reads continue but measurement counter stopped.
- `NO DATA` red: no fresh readings or status writer unavailable.
- `NO COUNTER` amber: older firmware cannot certify measurement progress.
- `STUCK? Ns` amber: old firmware, identical orientation for N seconds; suspicion.

Counter progress detects lack of new quaternion measurements; it does not prove
correct sensor calibration, physical mounting, or fusion accuracy. No automatic
sensor reset is implemented by this patch, and motion must not silently resume
after a stall.

## Verification

Native C++ test `progress_test.cpp`: first sample, duplicate timestamp, advancing
timestamp, sequence wrap, sensor timestamp restart.
ARM GCC 13.3.1 Debug build completed (276388 bytes FLASH). Linker reports an RWX
LOAD segment; physical firmware compatibility is still unverified.
42 focused Python tests passed for decoding, stale measurements, stationary
orientation with live counter, display state, polling and connection behavior.
Head runtime/display support installed; connected Zubr currently reports no
counter marker and retains the previous frozen IMU values. No motion launched.
