"""Head-button Wi-Fi controls for the robot's iwd interface."""
import asyncio
from pathlib import Path
import re

AP = 'Roki-Self'


async def iw(*args):
    proc = await asyncio.create_subprocess_exec('iwctl', *args, stdin=asyncio.subprocess.DEVNULL,
                                               stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT)
    try:
        out, _ = await asyncio.wait_for(proc.communicate(), 18)
    finally:
        if proc.returncode is None:
            proc.kill()
            await proc.wait()
    text = re.sub(r'\x1b\[[0-9;]*m', '', out.decode(errors='replace'))
    if proc.returncode:
        raise RuntimeError(text.strip() or 'iwd command failed')
    return text


async def mode():
    info = await iw('device', 'wlan0', 'show')
    return 'ap' if re.search(r'\bMode\s+ap\b', info) else 'station'


async def available():
    kind = await mode()
    try:
        await iw(kind, 'wlan0', 'scan')
    except RuntimeError as exc:
        if 'already in progress' not in str(exc).lower():
            raise
    for _ in range(20):
        await asyncio.sleep(.25)
        if not re.search(r'Scanning\s+yes', await iw(kind, 'wlan0', 'show')):
            break
    listing = await iw(kind, 'wlan0', 'get-networks')
    names = []
    for profile in Path('/var/lib/iwd').iterdir():
        if profile.suffix not in ('.open', '.psk', '.8021x'):
            continue
        name = profile.stem
        if name.startswith('='):
            try:
                name = bytes.fromhex(name[1:]).decode()
            except ValueError:
                continue
        if re.search(r'^\s*(?:>\s*)?' + re.escape(name) + r'\s+(?:open|psk|8021x)\s', listing, re.M):
            names.append(name)
        elif kind == "ap" and re.search(r"^\s*Name\s+" + re.escape(name) + r"\s*$", listing, re.M):
            names.append(name)
    return sorted(set(names))


async def choose(name):
    old = await mode()
    new = 'ap' if name is None else 'station'
    if old != new:
        await iw('device', 'wlan0', 'set-property', 'Mode', new)
    try:
        if name is None:
            if old != 'ap':
                await iw('ap', 'wlan0', 'start-profile', AP)
        else:
            if old == 'ap':
                await available()
            await iw('station', 'wlan0', 'connect', name)
    except Exception:
        if old != new:
            await iw('device', 'wlan0', 'set-property', 'Mode', old)
            if old == 'ap':
                await iw('ap', 'wlan0', 'start-profile', AP)
        raise

def preview(name):
    path = Path('/run/roki-wifi-selection')
    if name is None:
        path.unlink(missing_ok=True)
    else:
        tmp = path.with_suffix('.tmp')
        tmp.write_text(name)
        tmp.replace(path)
