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


@pytest.mark.parametrize('role', ['camera', 'detection', 'stream'])
def test_simple_game_recovers_media_peer_and_keeps_body_fault_fatal(tmp_path, role):
    from roki_ng.supervisor.lazy_worker import LazyWorkerPeer

    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
        s.mode = 'GAME'
        s.game.info.update(role='simple_football', running=True)
        w = LazyWorkerPeer(role, s.config | {'parameters': dict(s.params.values)}, s.worker_event, s.log)
        s.workers[role] = w
        old = SimpleNamespace(alive=False, close=AsyncMock())
        w.peer = old
        s._video_fault = AsyncMock()
        s.worker_event(role, 'worker.fault', {'error': 'process exited'})
        await asyncio.gather(*s.tasks)
        assert s.mode == 'GAME' and w.available and w.peer is None
        old.close.assert_awaited_once()
        if role != 'stream':
            assert s.game.vision_retry_at > 0
        # A late cleanup task from this generation cannot remove its replacement.
        w.peer = SimpleNamespace(alive=True)
        await w.retire_failed(old)
        assert w.alive
        s.worker_event('motherboard', 'worker.fault', {'error': 'body disconnected'})
        await asyncio.gather(*s.tasks)
        assert s.mode == 'FAULT'
    asyncio.run(run())


@pytest.mark.parametrize('role', ['camera', 'detection'])
def test_exited_media_process_can_restart_during_simple_game(tmp_path, role, monkeypatch):
    import socket
    import sys
    from roki_ng.supervisor.lazy_worker import LazyWorkerPeer

    if sys.platform == 'darwin':
        # Darwin has no Unix seqpacket socketpair. Preserve packet boundaries
        # for this process-lifetime test; production transport runs on Linux.
        monkeypatch.setattr(socket, 'SOCK_SEQPACKET', socket.SOCK_DGRAM)

    async def run():
        s = Supervisor({'simulate': True, 'state_dir': str(tmp_path), 'host': '127.0.0.1', 'port': 0})
        w = LazyWorkerPeer(role, s.config | {'parameters': {}}, s.worker_event, s.log)
        s.workers[role] = w
        s._video_fault = AsyncMock()
        try:
            await w.start()
            process = w.process
            s.mode = 'GAME'
            s.game.info.update(role='simple_football', running=True)
            process.kill()
            await asyncio.wait_for(process.wait(), 4)
            async with asyncio.timeout(4):
                while w.peer is not None:
                    await asyncio.sleep(.02)
            assert s.mode == 'GAME' and s.game.vision_retry_at
            await w.start()
            assert w.alive and w.process.pid != process.pid
            assert (await w.call(f'{role}.status')).get('error') in ('', None)
        finally:
            s.game.info.update(running=False)
            await w.close()
    asyncio.run(run())


def test_failed_camera_initialization_allows_another_explicit_start(tmp_path, monkeypatch):
    from roki_ng.supervisor import lazy_worker

    async def run():
        w = lazy_worker.LazyWorkerPeer('camera', {'state_dir': str(tmp_path)}, None, lambda *_: None)
        failed = SimpleNamespace(start=AsyncMock(side_effect=Fault('camera_fault', 'unavailable')),
                                 close=AsyncMock(), alive=False)
        ready = SimpleNamespace(start=AsyncMock(), alive=True)
        peers = iter((failed, ready))
        monkeypatch.setattr(lazy_worker, 'WorkerPeer', lambda *_: next(peers))
        with pytest.raises(Fault):
            await w.start()
        assert w.available and w.peer is None
        await w.start()
        assert w.peer is ready and w.alive
    asyncio.run(run())
