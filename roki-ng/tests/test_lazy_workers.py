import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.supervisor.supervisor import Supervisor, Session
from roki_ng.wire import Fault


def test_game_capture_release_respects_explicit_start_and_consumers(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        detection = {'running': False}
        local = {'running': False}
        video = {'active_streams': []}
        s.workers = {
            'camera': SimpleNamespace(alive=True),
            'detection': SimpleNamespace(call=AsyncMock(side_effect=lambda *a, **k: detection)),
            'localisation': SimpleNamespace(call=AsyncMock(side_effect=lambda *a, **k: local)),
            'stream': SimpleNamespace(call=AsyncMock(side_effect=lambda *a, **k: video)),
        }
        s._stop_capture = AsyncMock()
        s.capture_session = object()
        s.capture_explicit = True
        await s._retire_media(grace=0)
        s._stop_capture.assert_not_awaited()
        s.capture_explicit = False
        s.game.info['running'] = True
        await s._retire_media(grace=0)
        s._stop_capture.assert_not_awaited()
        s.game.info['running'] = False
        for consumer in (detection, local):
            consumer['running'] = True
            await s._retire_media(grace=0)
            s._stop_capture.assert_not_awaited()
            consumer['running'] = False
        video['active_streams'] = [{'name': 'camera', 'dependencies': []}]
        await s._retire_media(grace=0)
        s._stop_capture.assert_not_awaited()
        video['active_streams'] = []
        await s._retire_media(now=100)
        await s._retire_media(now=104)
        s._stop_capture.assert_not_awaited()
        await s._retire_media(now=106)
        s._stop_capture.assert_awaited_once()
    asyncio.run(run())


def test_catalogs_parameters_and_cleanup_do_not_spawn_media(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path), 'host': '127.0.0.1', 'port': 0})
        await s.start()
        try:
            session = Session("127.0.0.1", ('127.0.0.1', 5001))
            for role in ('camera', 'detection', 'localisation'):
                state = await s.workers[role].call(f'{role}.status')
                assert not state['running'] and state['state'] == 'stopped'
            await s.workers['camera'].call('camera.controls.list')
            await s.workers['camera'].call('camera.capabilities')
            await s._set_parameters({'camera.analogue_gain': 2., 'vision.orange_ball.l_min': 1})
            assert (await s.workers['camera'].call('camera.status'))['requested_controls']['gain'] == 2.
            assert await s._discover_video_sources()
            assert (await s._video_status('stream', session))['available']
            assert not (await s._video_status('camera', session))['available']
            await s._stop_capture()
            assert all(w.process is None for role, w in s.workers.items() if role != 'motherboard')
            assert s.workers['motherboard'].alive and s.mode == 'IDLE'
        finally:
            await s.close()
    asyncio.run(run())


def test_stream_lifetime_shared_receivers_and_settings_survive_restart(tmp_path):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path), 'host': '127.0.0.1', 'port': 0})
        await s.start()
        try:
            await s._discover_video_sources()
            w = s.workers['stream']
            await w.call('videostream.update', {'name': 'stream', 'settings': {'bitrate': 3000000}})
            assert w.process is None
            args = {'name': 'stream', 'operator_ip': 1, 'host': '127.0.0.1', 'rtp_port': 5004}
            first, _ = await asyncio.gather(w.call('videostream.subscribe', args),
                w.call('videostream.subscribe', args | {'operator_ip': 2, 'rtp_port': 5006}))
            process = w.process
            assert (await w.call('videostream.status', {'name': 'stream'}))['receivers'] == 2
            await w.call('videostream.unsubscribe', {'name': 'stream', 'operator_ip': 1})
            await w.retire_if_idle(grace=0)
            assert w.process is process and process.returncode is None
            await w.call('videostream.unsubscribe', {'name': 'stream', 'operator_ip': 2})
            await w.retire_if_idle(now=100)
            await w.call('videostream.status', {'name': 'stream'})
            await w.retire_if_idle(now=104)
            assert w.alive
            await w.retire_if_idle(now=106)
            assert w.process is None and process.returncode is not None
            state = await w.call('videostream.status', {'name': 'stream'})
            assert state['settings']['bitrate'] == 3000000 and not state['subscribed']
            second = await w.call('videostream.subscribe', args)
            assert second['ssrc'] != first['ssrc'] and second['settings']['bitrate'] == 3000000
            assert w.process.pid != process.pid and s.mode == 'IDLE'
        finally:
            await s.close()
    asyncio.run(run())


@pytest.mark.parametrize('role', ['camera', 'detection', 'localisation'])
def test_game_holds_idle_vision_process_and_shutdown_does_not_start_it(tmp_path, role):
    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path), 'host': '127.0.0.1', 'port': 0})
        await s.start()
        try:
            w = s.workers[role]
            # Explicitly initialise without hardware/processing, to exercise the
            # same idle state reached between game observation sessions.
            await asyncio.gather(w.start(), w.start())
            process = w.process
            await w.retire_if_idle(held=True, grace=0)
            assert w.process is process and w.alive
            await w.retire_if_idle(grace=0)
            assert w.process is None and process.returncode is not None
            assert s.mode == 'IDLE'
        finally:
            await s.close()
        with pytest.raises(Fault):
            await w.start()
    asyncio.run(run())


def test_localisation_explicit_start_recovers_after_process_exit(tmp_path, monkeypatch):
    from roki_ng.supervisor import lazy_worker

    async def run():
        w = lazy_worker.LazyWorkerPeer('localisation', {'state_dir': str(tmp_path)}, None, lambda *_: None)
        old = SimpleNamespace(alive=False, state={'state': 'fault', 'error': 'channel closed'}, close=AsyncMock())
        w.peer = old
        new = SimpleNamespace(start=AsyncMock(), call=AsyncMock(return_value={'running': True}))
        monkeypatch.setattr(lazy_worker, 'WorkerPeer', lambda *_: new)
        state = await w.call('localisation.status')
        assert state['state'] == 'fault' and not state['running']
        old.close.assert_not_awaited()
        request = {'prior': [0., 0., 0.]}
        assert (await w.call('localisation.start', request))['running']
        old.close.assert_awaited_once()
        new.start.assert_awaited_once()
        new.call.assert_awaited_once_with('localisation.start', request, urgent=False, timeout=3)
    asyncio.run(run())
