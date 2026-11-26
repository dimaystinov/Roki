"""Bounded live comparison via the existing operator protocol.

Run from repository root: PYTHONPATH=. python tools/tune_walk_tilt.py HOST
Requires a clear route and a supported upright robot. Never runs recovery.
Trial values are temporarily persisted by params.set; baseline is restored in
finally. Journal records restoration failures for explicit recovery. No value is
adopted automatically until two short comparisons and Short/Long validation pass.
"""
import argparse
import asyncio
import json
import math
import signal
import time
from contextlib import suppress
from pathlib import Path

from roki_ng.client import Client
from roki_ng.body.recovery import body_position
from roki_ng.body.walk_tuning import trial_score, search

KEY = 'walk.body_tilt_forward'


async def run(host, output):
    output.mkdir(parents=True, exist_ok=False)
    journal = {'host': host, 'key': KEY, 'trials': [], 'restored': False, 'adopted': False}
    def save():
        (output / 'journal.json').write_text(json.dumps(journal, indent=2, allow_nan=False))
    c = Client(host)
    connected = leased = False
    active = None
    pipe = None
    base = None

    async def snapshot():
        q = await c.request('data.snapshot', {'topic': 'body.imu'})
        try:
            if not q['valid'] or q['age_ms'] > 150:
                raise RuntimeError('Body IMU stale')
            if body_position(q['data']['quaternion_xyzw']) != 'upright':
                raise RuntimeError('Body no longer upright')
        except Exception:
            journal['rejected_imu'] = q
            save()
            raise
        return q

    async def trial(value, mode='custom'):
        nonlocal active
        # No inferred get-up: require verified upright BEFORE test.start.
        qs = []
        for _ in range(5):
            qs.append((await snapshot())['data']['quaternion_xyzw'])
            await asyncio.sleep(.1)
        if len(set(map(tuple, qs))) == 1:
            raise RuntimeError('Unchanging quaternion: source freshness unconfirmed')
        await c.request('params.set', {'key': KEY, 'value': value})
        item = {'value': value, 'mode': mode, 'started': time.time()}
        journal['trials'].append(item); save()
        args = {'name': 'run_test', 'mode': mode}
        if mode == 'custom':
            args.update(cycles=3, step_mm=24, side_mm=0, right_leg=True)
        job = await c.request('test.start', args)
        active = job['job_id']
        samples = []
        deadline = time.monotonic()+45
        with (output / ('trial-%02d.jsonl' % len(journal['trials']))).open('w') as f:
            while time.monotonic() < deadline:
                q = await snapshot()
                status = await c.request('job.status', {'job_id': active})
                correction = await c.request('data.snapshot', {'topic': 'body.stabilization'})
                f.write(json.dumps({'t': time.monotonic(), 'imu': q, 'job': status,
                                    'stabilization': correction})+'\n'); f.flush()
                if status.get('progress', 0) >= 1:
                    samples.append(tuple(math.degrees(v) for v in q['data']['rpy_rad'][:2]))
                if status['status'] in ('completed', 'failed', 'cancelled'):
                    active = None
                    if status['status'] != 'completed':
                        raise RuntimeError('Trial not completed: '+str(status))
                    break
                await asyncio.sleep(.07)
            else:
                raise RuntimeError('Trial timeout')
            # Include settling after final pose, do not score a walk only.
            for _ in range(20):
                q = await snapshot()
                samples.append(tuple(math.degrees(v) for v in q['data']['rpy_rad'][:2]))
                f.write(json.dumps({'t':time.monotonic(), 'imu':q, 'settling':True})+'\n')
                await asyncio.sleep(.1)
        score = trial_score(samples)
        item.update(score=score if math.isfinite(score) else None,
                    peak_deg=max(math.hypot(*a) for a in samples), completed=True)
        save(); print('TRIAL', item, flush=True)
        return score

    try:
        await c.connect(); connected = True
        if (await c.request('game.status'))['running']:
            raise RuntimeError('Game running')
        if (await c.request('data.snapshot', {'topic':'motion.state'}))['data']['active_job'] is not None:
            raise RuntimeError('Motion already active')
        base = (await c.request('params.get', {'key': KEY}))['value']
        journal['baseline'] = base; save()
        await c.request('control.acquire'); leased = True
        await c.request('mode.set', {'mode':'MANUAL'})
        await c.request('camera.start')
        pipe = await asyncio.create_subprocess_exec(
            'gst-launch-1.0','-eq','udpsrc','port=25146',
            'caps=application/x-rtp,media=video,encoding-name=H264,clock-rate=90000',
            '!','rtpjitterbuffer','latency=80','!','rtph264depay','!','h264parse',
            '!','matroskamux','!','filesink','location='+str(output/'video.mkv'),
            stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL)
        await c.request('videostream.subscribe', {'name':'camera','rtp_port':25146})
        result = await search(base, trial)
        journal['search'] = result
        journal['candidate'] = result['candidate']; save()
        print('CANDIDATE', result['candidate'],
              'baseline restored; Short/Long validation required', flush=True)
    except BaseException as exc:
        journal['error'] = str(exc) or type(exc).__name__; save()
        raise
    finally:
        if active is not None and connected:
            with suppress(Exception):
                await c.request('job.cancel', {'job_id': active})
                for _ in range(60):
                    status=await c.request('job.status', {'job_id': active})
                    if status['status'] in ('completed','failed','cancelled'):break
                    await asyncio.sleep(.1)
        if leased and base is not None:
            try:
                await c.request('params.set', {'key': KEY, 'value': base})
                journal['restored']=(await c.request('params.get', {'key': KEY}))['value']==base
            except Exception as exc:
                journal['restore_error']=str(exc)
            save()
        if connected:
            with suppress(Exception):await c.request('videostream.unsubscribe', {'name':'camera'})
            if leased:
                with suppress(Exception):await c.request('control.release')
            await c.close()
        if pipe and pipe.returncode is None:
            pipe.send_signal(signal.SIGINT)
            with suppress(asyncio.TimeoutError):await asyncio.wait_for(pipe.wait(),3)
            if pipe.returncode is None:pipe.kill();await pipe.wait()


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('host')
    p.add_argument('--output',type=Path,default=Path('output')/time.strftime('walk-auto-tune-%Y%m%d-%H%M%S'))
    a=p.parse_args()
    asyncio.run(run(a.host,a.output))
