"""On-demand media processes. Dormant controllers serve metadata only.

Constructors must remain resource-free: the allowlists below never execute
capture, processing, subscriptions or hardware operations in the supervisor.
"""
import asyncio
from copy import deepcopy
import time

from roki_ng.ipc import WorkerPeer
from roki_ng.wire import Fault


class LazyWorkerPeer:
    START = {'camera': {'camera.prepare'}, 'stream': {'videostream.subscribe'},
             'detection': {'detection.start', 'ball.start'},
             'localisation': {'localisation.start'}}
    PASSIVE = {
        'camera': {'camera.status', 'camera.capabilities', 'camera.controls.list', 'camera.stop'},
        'stream': {'state', 'sources.configure', 'videostream.status', 'videostream.capabilities',
                   'videostream.update', 'videostream.stop', 'videostream.stop_source',
                   'videostream.unsubscribe', 'videostream.detach_operator'},
        'detection': {'state', 'detection.status', 'detection.stop', 'ball.status', 'ball.stop',
                      'football.status', 'football.pose', 'football.decision', 'ball.geometry',
                      'source.status', 'source.stop'},
        'localisation': {'state', 'localisation.status', 'localisation.stop', 'source.status', 'source.stop'},
    }

    def __init__(self, role, config, event, log):
        self.role, self.config, self.event, self.log = role, deepcopy(config), event, log
        self.peer = None
        self.lock = asyncio.Lock()
        self.closed = False
        self.idle_since = None
        self.idle = self._controller()

    def _controller(self):
        if self.role == 'camera':
            from roki_ng.camera.worker import Camera as Controller
        elif self.role == 'stream':
            from roki_ng.stream.worker import Streams as Controller
        elif self.role == 'detection':
            from roki_ng.detection.worker import Detection as Controller
        else:
            from roki_ng.localisation.worker import Localisation as Controller
        return Controller(deepcopy(self.config), lambda *_: None, lambda *_: None)

    @property
    def alive(self):
        return self.peer is not None and self.peer.alive

    @property
    def available(self):
        return not self.closed and (self.peer is None or self.peer.alive)

    @property
    def process(self):
        return self.peer.process if self.peer and hasattr(self.peer, 'process') else None

    @property
    def state(self):
        return self.peer.state if self.peer else self.idle.state() | {'state': 'stopped'}

    @property
    def last_heartbeat(self):
        return self.peer.last_heartbeat if self.peer else time.monotonic()

    def _failed(self, reason):
        if self.peer:
            self.peer._failed(reason)

    async def retire_failed(self, peer):
        """Retire only the failed generation, leaving the on-demand endpoint usable."""
        async with self.lock:
            if peer is not None and self.peer is peer and not peer.alive:
                peer.event = lambda *_: None
                await peer.close()
                self.peer = None
                self.idle_since = None

    async def start(self):
        async with self.lock:
            await self._start()

    async def _start(self):
        if self.closed:
            raise Fault('worker_unavailable', self.role)
        if self.peer is not None:
            if not self.peer.alive:
                raise Fault('worker_unavailable', self.role)
            return
        peer = WorkerPeer(self.role, deepcopy(self.config), self.event, self.log)
        self.peer = peer
        try:
            await peer.start()
            if self.role == 'stream':
                await peer.call('sources.configure', {'items': [p.source for p in self.idle.pipelines.values()]})
                for name, p in self.idle.pipelines.items():
                    await peer.call('videostream.update', {'name': name, 'settings': p.settings})
        except BaseException:
            # No automatic replay of a request with an uncertain outcome.
            peer.event = lambda *_: None
            try:
                await peer.close()
            finally:
                # An initialization failure can precede the first worker.fault event.
                # The next explicit start must be able to create a fresh process.
                if self.role in ('camera', 'detection'):
                    self.peer = None
            raise
        self.idle_since = None
        self.log('supervisor', 'INFO', f'Worker {self.role} started on demand')

    async def call(self, op, body=None, *, urgent=False, timeout=3):
        body = body or {}
        async with self.lock:
            if self.closed:
                raise Fault('worker_unavailable', self.role)
            if self.role == 'localisation' and self.peer is not None and not self.peer.alive:
                if op in ('localisation.status', 'state'):
                    return self.idle.state() | self.peer.state
                if op == 'localisation.start':
                    await self.peer.close()
                    self.peer = None
            if self.peer is None:
                if op in self.START[self.role]:
                    await self._start()
                elif op == 'source.list':
                    return {'items': [self.idle.video_source()]}
                elif op == 'params.apply':
                    result = self.idle.command(op, body)
                    self.config['parameters'].update(body)
                    return result
                elif op in self.PASSIVE[self.role]:
                    result = self.idle.command(op, body)
                    if op in ('state', f'{self.role}.status'):
                        result = result | {'state': 'stopped'}
                    return result
                else:
                    raise Fault('not_ready', f'Start {self.role} before {op}')
            result = await self.peer.call(op, body, urgent=urgent, timeout=timeout)
            if op == 'params.apply':
                self.idle.command(op, body)
                self.config['parameters'].update(body)
            elif self.role == 'stream' and op in ('sources.configure', 'videostream.update'):
                self.idle.command(op, body)
            elif self.role == 'stream' and op == 'videostream.subscribe':
                self.idle.command('videostream.update', {'name': body['name'], 'settings': result['settings']})
            if op in self.START[self.role]:
                self.idle_since = None
            return result

    async def retire_if_idle(self, *, held=False, grace=5., now=None):
        # Serialised with calls, including launch and parameter changes. Polling
        # metadata does not prolong the lifetime of an otherwise idle process.
        if self.lock.locked() or not self.alive or self.closed:
            return
        async with self.lock:
            if not self.alive or self.closed:
                return
            op = 'state' if self.role == 'stream' else f'{self.role}.status'
            state = await self.peer.call(op)
            busy = state.get('running') or state.get('prepared') or state.get('active_streams')
            if self.role == 'detection':
                ball = await self.peer.call('ball.status')
                busy = busy or ball.get('running')
            now = time.monotonic() if now is None else now
            if held or busy:
                self.idle_since = None
                return
            if self.idle_since is None:
                self.idle_since = now
            if now - self.idle_since < grace:
                return
            if self.role == 'camera':
                controls = state['requested_controls']
                values = {'camera.exposure_us': controls['exposure_us'],
                          'camera.analogue_gain': controls['gain'],
                          'camera.ae_enabled': controls['ae_enabled'],
                          'camera.awb_enabled': controls['awb_enabled'],
                          'camera.white_balance.red_gain': controls['colour_gains'][0],
                          'camera.white_balance.blue_gain': controls['colour_gains'][1]}
                self.idle.command('params.apply', values)
                self.config['parameters'].update(values)
            # A planned exit is not worker.fault and must not revoke control.
            self.peer.event = lambda *_: None
            await self.peer.close()
            self.peer = None
            self.idle_since = None
            self.log('supervisor', 'INFO', f'Worker {self.role} stopped after idle grace')

    async def close(self):
        async with self.lock:
            self.closed = True
            if self.peer:
                self.peer.event = lambda *_: None
                await self.peer.close()
