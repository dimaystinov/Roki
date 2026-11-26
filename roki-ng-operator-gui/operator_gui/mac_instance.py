"""Per-login instance guard shared by the macOS launchers and app entry points."""
import contextlib
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import tempfile


@contextlib.contextmanager
def instance(role):
    path = Path(tempfile.gettempdir()) / f"roki-{os.getuid()}-{role}.lock"
    with path.open("a+") as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            handle.seek(0)
            pid = handle.read().strip()
            if pid.isdecimal():
                script = (
                    'ObjC.import("AppKit"); '
                    f'$.NSRunningApplication.runningApplicationWithProcessIdentifier({pid})'
                    '.activateWithOptions(3)'
                )
                subprocess.run(["osascript", "-l", "JavaScript", "-e", script],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            yield False
            return
        handle.seek(0)
        handle.truncate()
        handle.write(str(os.getpid()))
        handle.flush()
        yield True


if __name__ == "__main__":
    # Check both roles before creating either process in the paired launcher.
    for role in ("fpv", "simulator"):
        with instance(role) as available:
            if not available:
                sys.exit(1)
