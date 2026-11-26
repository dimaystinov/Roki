import asyncio

import pytest

from roki_ng.client import Client
from roki_ng.supervisor.supervisor import Supervisor
from roki_ng.wire import Fault, envelope, pack, unpack, udp_socket
from roki_ng.wire import normalize_body


def test_progress_sends_each_step_and_keeps_terminal_event(tmp_path):
    from roki_ng.supervisor.supervisor import Session
    server = Supervisor({'state_dir': str(tmp_path)})
    session = Session('127.0.0.1', ('127.0.0.1', 1234))
    packets = []
    server._send_raw = lambda packet, address: packets.append(unpack(packet))
    job = dict(job_id=1, stage='walking', progress=0, status='running')
    server.send(session, 'event', 'job.progress', job)
    for progress in range(1, 21):
        server.send(session, 'event', 'job.progress', job | {'progress': progress})
    server.send(session, 'event', 'job.progress', job | {'stage': 'finishing', 'progress': 20})
    server.send(session, 'event', 'job.completed', job | {'status': 'completed', 'progress': 20})
    assert [p['op'] for p in packets] == ['job.progress'] * 22 + ['job.completed']
    assert [p['body']['progress'] for p in packets[:21]] == list(range(21))
    assert packets[-1]['body']['progress'] == 20


def test_wire_imu_angles_are_array_without_mutating_internal_measurements(tmp_path):
    import math
    import time
    from types import SimpleNamespace
    server = Supervisor({'state_dir': str(tmp_path)})
    original = dict(valid=True, source_mono_ns=time.monotonic_ns(), roll_deg=90., pitch_deg=-45., yaw_rad=.3)
    server.body_telemetry['body.imu'] = original
    server.workers['motherboard'] = SimpleNamespace(alive=True)
    data = server._sample('body.imu')['data']
    assert data['rpy_rad'] == pytest.approx([math.pi / 2, -math.pi / 4, .3])
    assert not {'roll_deg', 'pitch_deg', 'yaw_rad'} & data.keys()
    assert original['roll_deg'] == 90. and 'rpy_rad' not in original


def test_empty_body_round_trip_preserves_explicit_state_and_errors():
    request = envelope('request', 'motion.stop_hard', {}, id=7)
    assert set(request) == {'kind', 'op', 'id'}
    assert normalize_body(unpack(pack(request)))['body'] == {}
    response = envelope('response', body={'result': {}}, id=7)
    assert response == {'kind': 'response', 'id': 7}
    assert normalize_body(unpack(pack(response)))['body'] == {'result': {}}
    for body in ({'result': {'owner': None, 'ball': None, 'viewers': []}},
                 {'error': {'code': 'busy', 'message': 'Busy', 'retryable': False}}):
        assert normalize_body(unpack(pack(envelope('response', body=body, id=7))))['body'] == body
    for invalid in (None, [], '', 0, False):
        with pytest.raises(Fault):
            normalize_body({'kind': 'request', 'op': 'motion.stop_hard', 'id': 7, 'body': invalid})


def test_endpoint_is_pinned_and_released_on_disconnect(tmp_path):
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path),
                             'host': '127.0.0.1', 'port': 0})
        await server.start()
        port = server.sock.getsockname()[1]
        owner = Client('127.0.0.1', port)
        other = udp_socket()
        other.bind(('127.0.0.1', 0))
        loop = asyncio.get_running_loop()
        try:
            welcome = await owner.connect()
            assert welcome['operator_ip'] == '127.0.0.1'
            assert 'boot_id' not in welcome
            connection = server.sessions['127.0.0.1']
            seen = connection.seen
            await loop.sock_sendto(other, pack(envelope('hello', id=1)), owner.address)
            raw, _ = await asyncio.wait_for(loop.sock_recvfrom(other, 1400), 1)
            reply = unpack(raw)
            assert set(reply) == {'kind', 'id', 'body'}
            assert reply['body']['error']['code'] == 'ip_in_use'
            assert connection.seen == seen
            await owner.close()
            for _ in range(100):
                if not server.sessions:
                    break
                await asyncio.sleep(.01)
            assert not server.sessions
            await loop.sock_sendto(other, pack(envelope('hello', id=1)), owner.address)
            raw, _ = await asyncio.wait_for(loop.sock_recvfrom(other, 1400), 1)
            assert unpack(raw)['kind'] == 'welcome'
            assert server.sessions['127.0.0.1'].address == other.getsockname()
        finally:
            if owner.sock.fileno() >= 0:
                await owner.close()
            other.close()
            await server.close()
    asyncio.run(run())


def test_job_uses_request_id_and_retry_does_not_start_again(tmp_path):
    async def run():
        server = Supervisor({'simulate': True, 'state_dir': str(tmp_path),
                             'host': '127.0.0.1', 'port': 0})
        await server.start()
        client = Client('127.0.0.1', server.sock.getsockname()[1])
        try:
            await client.connect()
            await client.request('control.acquire')
            await client.request('mode.set', {'mode': 'MANUAL'})
            assert await client.request('log.unsubscribe') == {}
            ident = client.id + 1
            result = await client.request('motion.pose', {'name': 'base_stand'})
            assert result['job_id'] == ident
            connection = server.sessions[client.operator_ip]
            internal = connection.jobs[ident]
            future = asyncio.get_running_loop().create_future()
            client.pending[ident] = future
            packet = envelope('request', 'motion.pose',
                              {'name': 'base_stand'}, id=ident)
            await asyncio.get_running_loop().sock_sendto(client.sock, pack(packet), client.address)
            reply = await asyncio.wait_for(future, 1)
            client.pending.pop(ident)
            assert reply['body']['result'] == result
            assert connection.jobs == {ident: internal}
            status = await client.request('job.status', {'job_id': ident})
            assert status['job_id'] == ident
            with pytest.raises(Fault, match='not retained'):
                await client.request('job.status', {'job_id': ident + 1000})
        finally:
            await client.close()
            await server.close()
    asyncio.run(run())
