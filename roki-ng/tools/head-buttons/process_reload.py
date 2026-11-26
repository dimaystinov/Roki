"""Dedicated BTN2 listener; remains available when roki-head is stopped."""
from contextlib import closing
import select
import subprocess
import time

from evdev import InputDevice, ecodes, list_devices


def say(text):
    subprocess.Popen(["espeak", "-ven-m1", "-a50", text])


def listen(dev):
    started = None
    exited = False
    dropped = False
    while True:
        if select.select([dev.fd], [], [], 0.1)[0]:
            for event in dev.read():
                if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_DROPPED:
                    started = None
                    dropped = True
                elif dropped:
                    if event.type == ecodes.EV_SYN and event.code == ecodes.SYN_REPORT:
                        dropped = False
                elif event.type == ecodes.EV_KEY and event.code == ecodes.KEY_F14:
                    if event.value == 1 and started is None:
                        started = time.monotonic()
                        pressed_timestamp = event.timestamp()
                        exited = False
                        say("Process re-load")
                        subprocess.run(["dinitctl", "stop", "roki-head"], check=True)
                    elif event.value == 0 and started is not None:
                        # Kernel timestamps preserve short taps even if stopping took time.
                        if event.timestamp() - pressed_timestamp <= 2 and not exited:
                            subprocess.run(["dinitctl", "start", "roki-head"], check=True)
                        elif not exited:
                            say("Exit from program")
                        started = None
        if started is not None and not exited and time.monotonic() - started > 2:
            if ecodes.KEY_F14 in dev.active_keys():
                say("Exit from program")
                exited = True


if __name__ == "__main__":
    for path in list_devices():
        with closing(InputDevice(path)) as device:
            if device.name == "roki-head-buttons":
                listen(device)
    raise SystemExit("roki-head-buttons unavailable")
