"""Operator UDP endpoint, leases, logs and fixed worker lifecycle."""

import asyncio
import collections
import math
from dataclasses import dataclass, field
import socket
import time
from roki_ng.identifiers import next_id

from roki_ng.ipc import WorkerPeer
from roki_ng.supervisor.lazy_worker import LazyWorkerPeer
from roki_ng.parameters import Parameters
from roki_ng.parameters import SCHEMA
from roki_ng.parameters import COLOUR_DEFAULTS
from roki_ng.parameters import validate_colour_ranges
from roki_ng.body.calibration import ROTATION_KEYS
from roki_ng.motherboard.bootstrap import Bootstrap
from roki_ng.wire import Fault
from roki_ng.wire import UDP_LIMIT
from roki_ng.wire import envelope
from roki_ng.wire import number
from roki_ng.wire import boolean
from roki_ng.wire import choice
from roki_ng.wire import pack
from roki_ng.wire import page
from roki_ng.wire import udp_socket
from roki_ng.wire import unpack
from roki_ng.wire import normalize_body

OPS = (
    "game.start", "game.stop", "game.status", "game.pause", "game.resume", "game.pickup",
    "localisation.start", "localisation.stop", "localisation.status",
    "camera.start", "camera.stop", "camera.status", "camera.controls.freeze",
    "camera.controls.list", "camera.controls.set", "camera.controls.save",
    "detection.list", "detection.start", "detection.stop", "detection.status", "detection.select",
    "session.heartbeat", "session.close", "system.status", "system.capabilities", "system.operations",
    "system.restart_stream_worker", "control.acquire", "control.release", "mode.set",
    "motion.drive", "motion.head", "motion.pose", "motion.jump", "motion.kick", "motion.slot",
    "motion.get_up", "motion.splits",
    "motion.slots", "motion.joints", "motion.stop_graceful", "motion.stop_hard", "job.status", "job.cancel",
    "test.list", "test.describe", "test.start", "test.measure", "camera.capabilities", "videostream.capabilities", "videostream.subscribe",
    "videostream.status", "videostream.update", "videostream.stop",
    "videostream.list", "videostream.unsubscribe",
    "params.keys", "params.describe", "params.get", "params.set",
    "data.list", "data.snapshot", "data.subscribe", "data.update", "data.unsubscribe",
    "log.sources", "log.snapshot", "log.subscribe", "log.update", "log.unsubscribe",
)
READ_ONLY = {
    "game.status",
    "localisation.status",
    "camera.status",
    "camera.controls.list", "videostream.list", "videostream.status",
    "videostream.subscribe", "videostream.unsubscribe",
    "detection.list", "detection.status",
    "motion.slots", "motion.joints", "test.list", "test.describe", "job.status", "camera.capabilities", "videostream.capabilities",
    "params.keys", "params.describe", "params.get", "system.status", "system.capabilities",
    "system.operations", "session.heartbeat", "session.close",
}
BODY_TOPICS = ("body.imu", "body.stabilization", "body.servos", "body.power")
BODY_TOPIC_MAX_AGE_MS = {"body.imu": 150, "body.stabilization": 500, "body.servos": 600, "body.power": 3000}
TOPIC_MAX_RATE_HZ = {"body.servos": 5, "body.power": 1}
TOPICS = ("game.state", "system.workers", "motion.state", "camera.state", "videostream.state", "detection.state", "localisation.state", *BODY_TOPICS)
LEVELS = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}
REQUEST_WINDOW = 128


@dataclass
class Session:
    id: str
    address: tuple
    seen: float = field(default_factory=time.monotonic)
    high_id: int = 0
    drive_sequence: int = 0
    cache: dict = field(default_factory=collections.OrderedDict)
    pending: dict = field(default_factory=dict)
    data: dict = field(default_factory=dict)
    logs: dict | None = None
    jobs: dict = field(default_factory=dict)
    closed: bool = False


class Supervisor:
    def __init__(self, config):
        self.config = config
        self.params = Parameters(config["state_dir"], config.get('robot_model'))
        self.config = config | {'robot_model': self.params.robot_model}
        self.sessions = {}
        self.owner = None
        self.motion_ready = True
        self.mode = "BOOTSTRAP"
        self.workers = {}
        self.job_routes = {}
        self.history = collections.deque(maxlen=1024)
        self.log_id = 0
        self.counters = collections.Counter()
        self.tasks = set()
        self.sock = None
        self.bootstrap = None
        self.motherboard_reset_task = None
        self.closing = False
        self.control_lock = asyncio.Lock()
        self.parameter_lock = asyncio.Lock()
        self.capture_lock = asyncio.Lock()
        self.capture_session = None
        self.capture_explicit = False
        self.capture_unused_since = None
        self.alignment_pending = collections.deque(maxlen=16)
        self.alignment_task = None
        self.head_menu = self.head_buttons = self.voice = None
        self.button_task = None
        self.video_sources = {}
        self.body_telemetry = {}
        self.body_watch = False
        self.body_servo_watch = False
        self.body_power_watch = False
        self.body_imu_watch = False
        self.body_watch_lock = asyncio.Lock()
        self.menu_context = object()  # Trusted local caller, never a network owner.
        self.menu_motion = False
        from roki_ng.supervisor.game.game import Goalkeeper
        self.game = Goalkeeper(self)
        self.football_diagnostic = False

    def spawn(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)
        task.add_done_callback(self._task_done)
        return task

    def _task_done(self, task):
        if not task.cancelled() and (error := task.exception()):
            self.log("supervisor", "ERROR", str(error))

    def log(self, source, level, message):
        message = str(message)
        raw = message.encode("utf-8", errors="replace")
        if len(raw) > 200:
            # Preserve long native diagnostics as consecutive bounded records.
            while raw:
                chunk = raw[:200].decode("utf-8", errors="ignore")
                self.log(source, level, chunk)
                raw = raw[len(chunk.encode("utf-8")):]
            return
        self.log_id += 1
        record = {"record_sequence": self.log_id, "monotonic_ns": time.monotonic_ns(),
                  "source": source[:64], "level": level,
                  "message": message}
        self.history.append(record)
        if self.params.values["logging.stdout_enabled"]:
            print(f"[{level}] {source}: {message}", flush=True)

    async def start(self):
        self.sock = udp_socket()
        self.sock.bind((self.config.get("host", "0.0.0.0"), self.config.get("port", 8093)))
        self.spawn(self._receive())
        self.spawn(self._maintenance())
        self.log("supervisor", "INFO", f"UDP control: {self.sock.getsockname()}")
        try:
            if not self.config.get("simulate") and not self.config.get("skip_bootstrap"):
                self.bootstrap = Bootstrap(self.config)
                await asyncio.to_thread(self.bootstrap.start)
            for role in ("motherboard", "stream", "camera", "detection", "localisation"):
                factory = WorkerPeer if role == 'motherboard' else LazyWorkerPeer
                worker = factory(role, self.config | {"parameters": dict(self.params.values)}, self.worker_event, self.log)
                self.workers[role] = worker
                if role == 'motherboard':
                    await worker.start()
            if not self.workers['motherboard'].alive:
                raise RuntimeError("Worker failed during startup")
            self.mode = "IDLE"
            self.log("supervisor", "INFO", "Runtime ready; media workers will start on demand")
            self.spawn(self._media_lifecycle())
        except Exception as exc:
            self.mode = "FAULT"
            self.log("supervisor", "ERROR", f"Startup failed: {exc}")
        if self.mode == "IDLE" and not self.config.get("simulate") and not self.config.get("no_head_menu"):
            await self._start_head_menu()

    async def _start_head_menu(self):
        from roki_ng.supervisor.head_menu import HeadButtons
        from roki_ng.supervisor.head_menu import HeadMenu
        from roki_ng.supervisor.head_menu import Voice

        log = lambda level, text: self.log("head-menu", level, text)
        self.voice = Voice(log)
        self.head_menu = HeadMenu(self._head_command, self._head_release, self.voice.say, log)
        self.head_buttons = HeadButtons(self.head_menu, log)
        try:
            self.head_buttons.open()
        except (ImportError, OSError) as exc:
            log("ERROR", f"Menu unavailable: {exc}")
            await self.voice.close()
            self.head_menu = self.head_buttons = self.voice = None
            return
        self.button_task = self.spawn(self.head_buttons.run())
        self.voice.say("Ready")

    async def _head_command(self, op, body):
        if op == 'menu.own_goal':
            colour = choice(body, 'colour', None, ('yellow', 'blue'))
            matches = [i for i in (0, 1) if self.params.values[f'field.goal.{i}']['colour'] == colour]
            if len(matches) != 1:
                raise Fault('invalid_argument', 'Goal colour must identify exactly one mapped goal')
            if self.game.info['running'] and not self.game.info.get('pickup_ready'):
                raise Fault('busy', 'Stop game or complete Pick up before selecting goal')
            # Menu navigation and its non-motion selections must never touch
            # the body control lease.  A GUI owner remains authoritative; the
            # head may not silently acquire or interrupt its controls merely
            # because the operator browsed the goal-colour item.
            await self._set_parameters({'match.own_goal': matches[0]})
            return {'colour': colour, 'own_goal': matches[0]}
        if op == 'game.pickup':
            # Trusted physical operator input has priority over the GUI lease.
            # Network game.pickup still goes through require_control.
            return await self.game.pickup(body)
        if op == 'game.stop':
            result = await self.game.stop()
            await self._head_release()
            return result
        if op in ("test.start", "game.start"):
            if self.game.info["running"] and not (op == "game.start" and self.game.pickup_requested):
                raise Fault("busy", "Stop game before starting another task")
            if self.menu_motion:
                raise Fault('busy', 'Menu motion already running')
            self.require_motion_ready()
            self.menu_motion = True
            try:
                # Physical menu commands bypass network ownership, but still use
                # the same body busy checks and game/stop barriers.
                if not (op == 'game.start' and self.game.pickup_requested):
                    await self.workers["motherboard"].call("control.acquire", urgent=True)
                    await self.dispatch(self.menu_context, "mode.set", {"mode": "MANUAL"})
                result = await self.dispatch(self.menu_context, op, body)
                if op == 'game.start':
                    self.menu_motion = False  # Game owns its own lifetime.
                return result
            except BaseException:
                await self._head_release()
                raise
        return await self.dispatch(self.menu_context, op, body)

    async def _head_release(self):
        if not self.menu_motion:
            return
        self.menu_motion = False
        if self.owner is None and not self.game.info['running']:
            await self.workers["motherboard"].call("control.release", urgent=True)
            if self.mode != 'FAULT':
                self.mode = 'IDLE'

    def _game_media_recovery(self, role):
        return (role in ('camera', 'detection', 'stream') and self.mode == 'GAME'
                and self.game.info.get('running') and self.game.info.get('role') == 'simple_football')

    def worker_event(self, role, op, body):
        if role == 'motherboard' and self.game.simple:
            if op == 'body.telemetry':
                self.game.simple.check_goal_memory(body.get('body.imu', {}))
            elif op in ('worker.fault', 'motherboard.reset_required', 'job.failed', 'job.cancelled') or (
                    op == 'body.connection' and not body.get('body_connected')):
                self.game.simple.forget_goal(op)
        if role == 'motherboard' and op in ('job.progress', 'body.connection') and self.football_diagnostic:
            if op == 'job.progress' or body.get('active_job') or not body.get('body_connected'):
                self.football_diagnostic = False
                self.spawn(self.workers['detection'].call('football.pose', {'active': False,
                           'note': 'Body moved or disconnected: restart football diagnostics while stationary'}))
        if role == 'motherboard' and op == 'motherboard.reset_required':
            if not self.closing and (self.motherboard_reset_task is None or self.motherboard_reset_task.done()):
                self.motherboard_reset_task = self.spawn(self._reset_motherboard())
            return
        if not self.closing and ((role == 'stream' and op in ('videostream.failed', 'videostream.stopped', 'worker.fault'))
                                 or (role in self.video_sources and op in (f'{role}.fault', 'worker.fault'))):
            self.spawn(self._video_fault(role, op))
        if role == "motherboard" and op == "body.telemetry":
            self.body_telemetry = body
            return  # Only requested datastreams, never unsolicited operator events.
        if self.head_menu and role == "motherboard" and not self.closing:
            if op in ("job.completed", "job.cancelled", "job.failed"):
                self.spawn(self.head_menu._finish(body))
            elif op == "worker.fault":
                self.spawn(self.head_menu.worker_failed())
        if role == "motherboard" and op == "imu.alignment":
            if self.capture_session is not None and not self.closing:
                # Startup needs recent timestamp pairs, not every historical edge.
                # Keep receiving while a slow media start holds capture_lock.
                self.alignment_pending.append((self.capture_session, body))
                if self.alignment_task is None or self.alignment_task.done():
                    self.alignment_task = self.spawn(self._forward_alignment())
            return  # Internal startup metadata, never an operator datastream.
        if op in ("camera.fault", "imu.fault") and not self.closing:
            self.spawn(self._capture_fault(self.capture_session))
        if role == "motherboard" and op == "calibration.ready":
            self.spawn(self._save_calibration(body))
            return
        if op == "worker.fault" and not self.closing:
            if role == "localisation":
                self.log(role, "ERROR", body.get("error", "Localisation worker failed"))
                return  # Diagnostic failure must not revoke manual control.
            if self._game_media_recovery(role):
                worker = self.workers.get(role)
                if isinstance(worker, LazyWorkerPeer):
                    self.spawn(worker.retire_failed(worker.peer))
                if role in ('camera', 'detection'):
                    self.game.vision_retry_at = time.monotonic() + 2
                if role == 'camera':
                    self.spawn(self._capture_fault(self.capture_session))
                self.log(role, 'WARNING', f'Game continues after media failure: {body.get("error", role)}')
                return
            self.mode = "FAULT"
            self.log(role, "ERROR", body.get("error", "worker failed"))
            if role in ("camera", "motherboard"):
                self.spawn(self._capture_fault(self.capture_session))
            if self.owner and self.motion_ready:
                owner = self.sessions.get(self.owner)
                if owner:
                    self.spawn(self._release(owner))
        if role == 'motherboard' and op.startswith('job.') and body.get('operation', '').startswith('game.'):
            # Full confirmed segments are local IPC data for odometry. A long
            # original manoeuvre must not overflow operator event datagrams.
            body = {key: value for key, value in body.items() if key != 'segments'}
        for session in list(self.sessions.values()):
            if op.startswith("job.") and session.id != self.owner:
                continue
            self.send(session, "event", op, body)

    def send(self, session, kind, op, body, ident=0):
        if session.closed:
            return None
        body = self._public_jobs(session, body)
        if op == 'job.progress':
            body = {k: v for k, v in body.items() if k in (
                'job_id', 'operation', 'status', 'progress', 'stage', 'execution_uncertain')}
        message = envelope(kind, op, body, id=ident)
        try:
            data = pack(message)
        except Fault:
            self.counters["oversize"] += 1
            if kind != "response":
                return None
            message["body"] = {"error": Fault("too_large", "Response exceeds budget; reduce page size").as_dict()}
            data = pack(message)
        self._send_raw(data, session.address)
        return data

    def _public_jobs(self, session, value):
        if isinstance(value, dict):
            result = {}
            for key, item in value.items():
                if key in ('job_id', 'active_job') and type(item) is int:
                    route = self.job_routes.get(item)
                    result[key] = route[1] if route and route[0] is session else item
                else:
                    result[key] = self._public_jobs(session, item)
            return result
        if isinstance(value, list):
            return [self._public_jobs(session, item) for item in value]
        return value

    def _new_job(self, session, request_id):
        if request_id is None:
            return None
        ident = next_id()
        session.jobs[request_id] = ident
        self.job_routes[ident] = (session, request_id)
        while len(session.jobs) > 64:
            oldest = next(iter(session.jobs))
            self.job_routes.pop(session.jobs.pop(oldest), None)
        return ident

    def _send_raw(self, data, address):
        try:
            self.sock.sendto(data, address)
        except OSError:
            self.counters["send_errors"] += 1

    async def _receive(self):
        loop = asyncio.get_running_loop()
        while True:
            raw, address = await loop.sock_recvfrom(self.sock, UDP_LIMIT + 1)
            try:
                message = normalize_body(unpack(raw))
                kind = message.get("kind")
                fields = {'kind', 'body'}
                if kind in ('hello', 'request'):
                    fields.add('id')
                if kind in ('request', 'sample'):
                    fields.add('op')
                if kind == 'sample':
                    fields.add('sequence')
                if kind not in ('hello', 'request', 'sample') or set(message) != fields:
                    raise Fault('bad_message', 'Invalid envelope fields')
                if kind == "hello":
                    self._hello(message, address)
                    continue
                session = self.sessions.get(address[0])
                if session is None or session.closed or session.address != address:
                    self.counters["invalid_session"] += 1
                    continue
                if not isinstance(message.get("body"), dict) or not isinstance(message.get("op"), str):
                    raise Fault("bad_message", "Expected op string and body map")
                session.seen = time.monotonic()
                if kind == "sample" and message["op"] == "motion.drive":
                    sequence = number(message, "sequence", 0, 1, 2**64 - 1, True)
                    if sequence <= session.drive_sequence:
                        continue
                    session.drive_sequence = sequence
                    # At most one forwarded drive in flight; new packets replace it.
                    session.latest_drive = message
                    if not getattr(session, "drive_task", None) or session.drive_task.done():
                        session.drive_task = self.spawn(self._drive(session))
                elif kind == "request":
                    ident = number(message, "id", 0, 1, 2**64 - 1, True)
                    if ident in session.cache:
                        old_op, old_body, data = session.cache[ident]
                        if (old_op, old_body) == (message["op"], message["body"]):
                            self._send_raw(data, address)
                        else:
                            self.send(session, "response", message["op"], {"error": Fault("id_conflict", "ID reused with different request").as_dict()}, ident)
                        continue
                    if ident in session.pending:
                        original = session.pending[ident]
                        if (original["op"], original["body"]) != (message["op"], message["body"]):
                            self.send(session, "response", message["op"], {"error": Fault("id_conflict", "ID reused with different request").as_dict()}, ident)
                        continue
                    if ident <= session.high_id - REQUEST_WINDOW:
                        self.send(session, "response", message["op"], {"error": Fault("stale_request", "ID is older than retained request window").as_dict()}, ident)
                        continue
                    session.high_id = max(session.high_id, ident)
                    # Evict by ID, not completion order: every accepted ID within
                    # the window must remain either pending or cached.
                    for old_id in list(session.cache):
                        if old_id <= session.high_id - REQUEST_WINDOW:
                            del session.cache[old_id]
                    if len(session.pending) >= 8 or len(self.tasks) >= 64:
                        # Cache busy as well: retries must not execute it later.
                        data = self.send(session, "response", message["op"], {"error": Fault("busy", "Too many requests", True).as_dict()}, ident)
                        self._cache(session, message, data)
                        continue
                    session.pending[ident] = message
                    self.spawn(self._request(session, message))
                else:
                    raise Fault("bad_message", "Unsupported message kind")
            except (Fault, TypeError, ValueError):
                self.counters["bad_packets"] += 1

    def _hello(self, message, address):
        body = message.get("body", {})
        if body != {}:
            raise Fault("bad_message", "hello body must be empty")
        ident = number(message, "id", 0, 1, 2**64 - 1, True)
        session = self.sessions.get(address[0])
        if session is not None and (session.closed or session.address != address):
            self._send_raw(pack(envelope("response", body={"error": Fault("ip_in_use", "This IP already has an active connection").as_dict()}, id=ident)), address)
            return
        if session is None:
            if len(self.sessions) >= 4:
                return
            session = Session(address[0], address)
            self.sessions[session.id] = session
            self.log("supervisor", "INFO", f"Operator connected: {address[0]}:{address[1]}")
        session.seen = time.monotonic()
        self.send(session, "welcome", "hello", {
            "robot_id": self.config.get("robot_id", socket.gethostname()), "operator_ip": address[0],
            "heartbeat_ms": 500, "session_timeout_ms": 2000, "drive_timeout_ms": 700,
            "state": self.mode, "max_datagram": UDP_LIMIT,
        }, ident)

    def _cache(self, session, message, data):
        if data and message["id"] > session.high_id - REQUEST_WINDOW:
            session.cache[message["id"]] = (message["op"], message["body"], data)

    async def _request(self, session, message):
        try:
            result = await self.dispatch(session, message["op"], message["body"], request_id=message['id'])
            body = {"result": result}
        except Fault as exc:
            body = {"error": exc.as_dict()}
        except Exception as exc:
            self.log("supervisor", "ERROR", f"{message['op']}: {exc}")
            body = {"error": Fault("internal_error", str(exc)).as_dict()}
        data = self.send(session, "response", message["op"], body, message["id"])
        self._cache(session, message, data)
        session.pending.pop(message["id"], None)
        if message["op"] == "session.close":
            await self._expire(session)

    def require_control(self, session, body):
        if session is self.menu_context:
            return
        if session is None or session.closed or self.owner != session.id:
            raise Fault("not_owner", "A valid control lease is required")

    def require_motion_ready(self):
        if not self.motion_ready:
            raise Fault("stop_unconfirmed", "Retry forced control acquisition to confirm the body queue reset", True)

    def require_game_idle(self):
        if self.game.info['running'] or self.game.starting:
            raise Fault('busy', 'Stop goalkeeper before changing motion or capture configuration')

    def require_game_parameters(self, values):
        if self.game.info['running'] or self.game.starting:
            if any(SCHEMA[key][4] not in ('live', 'next_frame', 'next_request', 'next_localisation')
                   for key in values):
                raise Fault('busy', 'Stop goalkeeper before changing motion or game geometry parameters')

    async def _take_control(self, session):
        """Called under control_lock. Revoke first, then reset via urgent IPC."""
        try:
            await self.game.stop('control_takeover', hard=True)
        except Fault as exc:
            # A failed autonomous stop must not strand ownership with a lost
            # operator. Grant the new lease below, but keep motion blocked until
            # the independent takeover reset is acknowledged.
            self.log('supervisor', 'ERROR', f'Game stop before takeover unconfirmed: {exc}')
        if self.head_menu:
            self.head_menu.taken_over()
        self.menu_motion = False
        previous = self.owner
        old = self.sessions.get(previous)
        self.owner = session.id
        self.motion_ready = False
        if old:
            old.latest_drive = None
            if old.id != session.id:
                self.send(old, "event", "control.revoked", {"reason": "operator_takeover"})
        self.log("supervisor", "WARNING", f"Forced control takeover: {previous} -> {session.id}")
        error = None
        worker = self.workers.get("motherboard")
        try:
            if not worker or not worker.alive:
                raise Fault("worker_unavailable", "motherboard")
            result = await worker.call("control.takeover", urgent=True)
            if result.get('stop_confirmed') is not True:
                raise Fault('stop_unconfirmed', 'Takeover queue reset not confirmed')
            self.game.queue_stopped()
            await self.game.stop('control_takeover')
            self.motion_ready = True
        except Fault as exc:
            error = exc.as_dict()
            self.log("supervisor", "ERROR", f"Takeover stop unconfirmed: {exc}")
        if session.closed:
            self.owner = None
            if worker and worker.alive:
                try:
                    await worker.call("control.release", urgent=True)
                except Fault:
                    self.motion_ready = False
            raise Fault("session_expired", "Session closed during acquisition")
        return {"owner": self.owner, "motion_ready": self.motion_ready,
                "stop_confirmed": self.motion_ready, "stop_error": error}

    async def _drive(self, session):
        while getattr(session, "latest_drive", None):
            message, session.latest_drive = session.latest_drive, None
            try:
                self.require_control(session, message["body"])
                self.require_motion_ready()
                if self.mode != "MANUAL" or self.menu_motion:
                    raise Fault("invalid_state", "Manual drive unavailable during menu motion")
                if self.parameter_lock.locked():
                    raise Fault("busy", "Parameters are being saved", True)
                async with self.parameter_lock:
                    self.require_control(session, message["body"])
                    if self.mode != "MANUAL" or self.menu_motion:
                        raise Fault("busy", "Autonomous motion active")
                    await self.workers["motherboard"].call("motion.drive", message["body"])
            except Fault as exc:
                # Repeated drive failures are rate-limited, not ACKed at driving frequency.
                if time.monotonic() - getattr(session, "last_drive_error", 0) > 1:
                    session.last_drive_error = time.monotonic()
                    self.send(session, "event", "motion.rejected", {"error": exc.as_dict()})

    async def dispatch(self, session, op, body, *, _from_game=False, request_id=None):
        if request_id is not None and op in ('job.status', 'job.cancel', 'test.measure'):
            ident = body.get('job_id')
            if type(ident) is not int:
                raise Fault('invalid_argument', 'Expected numeric job_id')
            internal = session.jobs.get(ident) if ident > 0 else ident
            if internal is None:
                raise Fault('not_found', 'Job is not retained for this connection')
            body = body | {'job_id': internal}
        if _from_game and (op != 'camera.start' or not self.game.info['running'] or self.game.stop_requested):
            raise Fault('cancelled', 'No active game capture request')
        if op == "game.status":
            return self.game.state()
        if op in ("game.start", "game.stop", "game.pause", "game.resume", "game.pickup"):
            self.require_control(session, body)
            if op == "game.stop":
                return await self.game.stop()
            if op == "game.pause":
                return await self.game.pause()
            if op == "game.resume":
                return await self.game.resume()
            if op == "game.pickup":
                return await self.game.pickup(body)
            generation = self.game.generation
            async with self.parameter_lock:
                self.require_control(session, body)
                if generation != self.game.generation:
                    raise Fault('cancelled', 'Game start discarded at stop barrier')
                result = await self.game.start(session, body, job_id=self._new_job(session, request_id))
                self.football_diagnostic = False
                return result
        if op in ('motion.stop_hard', 'motion.stop_graceful') or (op == 'mode.set' and body.get('mode') == 'IDLE'):
            self.require_control(session, body)
            await self.game.stop('operator_stop', hard=op == 'motion.stop_hard')
        elif (self.game.info['running'] or self.game.starting) and not _from_game:
            if (op.startswith(('motion.', 'test.')) and op not in READ_ONLY) or op in (
                    'camera.start', 'camera.stop', 'mode.set', 'system.restart_stream_worker', 'job.cancel'):
                raise Fault('busy', 'Stop goalkeeper before changing motion or capture configuration')
        if op not in OPS:
            raise Fault("not_supported", op)
        if op == "session.heartbeat":
            return {"state": self.mode, "owner": self.owner}
        if op == "session.close":
            return {"closing": True}
        if op == "system.operations":
            return page(list(OPS), body, 12)
        if op == "system.capabilities":
            return {"modes": ["IDLE", "MANUAL"],
                    "body": ["software_slots", "walk", "jump", "kick", "head", "tests"],
                    "data_topics": list(TOPICS),
                    "capture_backends": ["libcamera-iceoryx2"],
                    "detectors": ["colour_blobs", "ball"],
                    "localisation": {"mode": "stationary_correction", "motion_pose": True},
                    "game": {"strategies": ["FIRA_penalty_Goalkeeper", "forward", "ball_kick_test", "simple_football", "danger_football", "simple_goalkeeper"], "entries": ["center", "left", "right"]},
                    "future": ["osd", "servo-parameters"],
                    "hardware_slots": False, "simulated": bool(self.config.get("simulate"))}
        if op == "system.status":
            return {"state": self.mode, "owner": self.owner,
                    "motion_ready": self.motion_ready,
                    "workers": {k: {"alive": w.alive, "state": w.state.get("state")} for k, w in self.workers.items()},
                    "counters": dict(self.counters)}
        if op == "control.acquire":
            force = boolean(body, "force", False)
            async with self.control_lock:
                if session.closed:
                    raise Fault("session_expired", "Session closed")
                if force:
                    return await self._take_control(session)
                body_worker = self.workers.get("motherboard")
                if self.mode == "BOOTSTRAP" or not body_worker or not body_worker.alive:
                    raise Fault("not_ready", self.mode)
                if self.owner and self.owner != session.id:
                    raise Fault("busy", "Another operator owns control")
                if self.owner is None:
                    self.require_motion_ready()
                    if not self.game.info['running'] and not self.menu_motion:
                        await self.workers["motherboard"].call("control.acquire", urgent=True)
                    if session.closed:
                        if not self.game.info['running']:
                            await self.workers["motherboard"].call("control.release", urgent=True)
                        raise Fault("session_expired", "Session closed during acquisition")
                    self.owner = session.id
                return {"owner": self.owner, "motion_ready": self.motion_ready}
        if op == "control.release":
            self.require_control(session, body)
            await self._release(session)
            return {"released": True}
        if op.startswith("data."):
            return await self._data(session, op, body)
        if op.startswith("log."):
            return self._logs(session, op, body)
        if op == "params.keys":
            prefix = body.get("prefix", "")
            if not isinstance(prefix, str):
                raise Fault("invalid_argument", "prefix must be a string")
            return page(sorted(k for k in SCHEMA if k.startswith(prefix)), body)
        if op == "params.get" and "keys" in body:
            keys = body["keys"]
            if ("key" in body or not isinstance(keys, list) or not 1 <= len(keys) <= 16
                    or any(not isinstance(k, str) for k in keys) or len(set(keys)) != len(keys)):
                raise Fault("invalid_argument", "Provide 1..16 distinct keys, without key")
            async with self.parameter_lock:
                for key in keys:
                    self.params.describe(key)
                return {"values": {key: self.params.values[key] for key in keys}}
        if op in ("params.get", "params.describe"):
            key = body.get("key")
            meta = self.params.describe(key)
            return meta if op.endswith("describe") else {"key": key, "value": self.params.values[key]}
        if op not in READ_ONLY and not _from_game:
            self.require_control(session, body)
        if op == 'camera.controls.list':
            return await self.workers['camera'].call(op, body)
        if op in ('camera.controls.set', 'camera.controls.save'):
            from roki_ng.camera.worker import CONTROL_KEYS
            from roki_ng.camera.worker import WB_KEYS
            values = body.get('values')
            if not isinstance(values, dict) or not values or set(values) - (set(CONTROL_KEYS) | set(WB_KEYS)):
                raise Fault('invalid_argument', 'Provide camera control values using camera.* keys')
            if op.endswith('.save'):
                return {'values': await self._set_parameters(values), 'saved': True}
            async with self.parameter_lock:
                values = {key:self.params.validate(key,value) for key,value in values.items()}
                self.require_game_parameters(values)
                self.require_control(session, body)
                await self.workers['camera'].call('params.apply', values)
                return {'values': values, 'saved': False}
        if op == "params.set":
            if "values" in body:
                values, expected = body["values"], body.get("expected_values")
                if any(k in body for k in ("key", "value", "expected_value")):
                    raise Fault("invalid_argument", "Do not mix single and batch formats")
                if not isinstance(values, dict) or not 1 <= len(values) <= 16:
                    raise Fault("invalid_argument", "Provide 1..16 scalar parameters")
                if any(k not in SCHEMA or SCHEMA[k][0] == "object" for k in values):
                    raise Fault("invalid_argument", "Batch supports known scalar parameters only")
                if expected is not None and (not isinstance(expected, dict) or set(expected) != set(values)):
                    raise Fault("invalid_argument", "expected_values must cover exactly values")
                return {"values": await self._set_parameters(values, expected=expected)}
            if "expected_values" in body:
                raise Fault("invalid_argument", "expected_values requires values")
            key, value = body.get("key"), body.get("value")
            expected = {key: body['expected_value']} if 'expected_value' in body else None
            values = await self._set_parameters({key: value}, expected=expected)
            return {"key": key, "value": values[key], "apply": SCHEMA[key][4]}
        if op == 'camera.controls.freeze':
            from roki_ng.wire import choice
            group=choice(body,'group','all',('all','exposure','white_balance'))
            async with self.capture_lock:
                self.require_control(session,body)
                state=await self.workers['camera'].call('camera.status')
                measured=state.get('measured_controls');age=state.get('measured_age_ms')
                if not state.get('running') or not measured or age is None or age>1000:
                    raise Fault('not_ready','Fresh runtime camera metadata required')
                values={}
                if group in ('all','exposure'):
                    values.update({'camera.ae_enabled':False,'camera.exposure_us':measured.get('exposure_us'),
                                   'camera.analogue_gain':measured.get('gain')})
                if group in ('all','white_balance'):
                    gains=measured.get('colour_gains')
                    if not isinstance(gains,(list,tuple)) or len(gains)!=2:
                        raise Fault('not_ready','ColourGains metadata unavailable')
                    values.update({'camera.awb_enabled':False,'camera.white_balance.red_gain':gains[0],
                                   'camera.white_balance.blue_gain':gains[1]})
                async with self.parameter_lock:
                    values = {key:self.params.validate(key,value) for key,value in values.items()}
                    self.require_game_parameters(values)
                    self.require_control(session,body)
                    await self.workers['camera'].call('params.apply', values)
                return {'values':values,'saved':False,'source_sequence':measured['sequence']}
        if op == "test.measure":
            job = await self.workers["motherboard"].call("job.status", {"job_id": body.get("job_id")})
            if job["operation"] != "test.start" or job["status"] != "completed":
                raise Fault("invalid_state", "Measurements require a successfully completed test")
            values = body.get("values")
            expected = set(job.get("manual_parameters", ()))
            if not expected or not isinstance(values, dict) or set(values) != expected:
                raise Fault("invalid_argument", "Provide exactly the job's manual_parameters")
            # Spot-run measures residual drift with the corrections captured at
            # test start. Re-submitting this measurement must be idempotent.
            values = {key: self.params.validate(key, value) + job.get("measurement_baseline", {}).get(key, 0)
                      for key, value in values.items()}
            if 'run_command_mm' in job:
                run_key, = expected
                if job['run_command_mm'] <= 0:
                    raise Fault('invalid_state', 'No completed forward movement to calibrate')
                values[run_key.removesuffix('_mm') + '_command_mm'] = job['run_command_mm']
            return {"job_id": job["job_id"], "saved": await self._set_parameters(values)}
        if op == "mode.set":
            if body.get("mode") not in ("IDLE", "MANUAL"):
                raise Fault("not_supported", "Only IDLE and MANUAL are implemented")
            if self.mode == "FAULT":
                raise Fault("invalid_state", "Resolve worker fault first")
            if body["mode"] == "IDLE":
                await self.workers["motherboard"].call("motion.stop_hard", urgent=True)
            self.mode = body["mode"]
            return {"state": self.mode}
        if op == "system.restart_stream_worker":
            async with self.capture_lock:
                old_worker = self.workers["stream"]
                # An intentional exit is not a loss-of-control fault.
                old_worker.event = lambda *_: None
                await old_worker.close()
                await self._sync_video_sources(force_off=True)
                worker = LazyWorkerPeer("stream", self.config | {'parameters': dict(self.params.values)}, self.worker_event, self.log)
                if isinstance(old_worker, LazyWorkerPeer):
                    worker.idle = old_worker.idle
                self.workers["stream"] = worker
                await worker.start()
            if self.workers["motherboard"].alive:
                self.mode = "IDLE"
            return {"restarted": "stream"}
        if op.startswith(("motion.", "test.", "job.")):
            if op == "motion.drive":
                raise Fault("bad_message", "Use kind=sample for motion.drive")
            if op not in READ_ONLY and op not in ("motion.stop_hard", "motion.stop_graceful", "job.cancel") and self.mode != "MANUAL":
                raise Fault("invalid_state", "Select MANUAL mode")
            urgent = op in ("motion.stop_hard", "motion.stop_graceful", "job.cancel")
            if urgent or op in READ_ONLY:
                return await self.workers["motherboard"].call(op, body, urgent=urgent)
            self.require_motion_ready()
            async with self.parameter_lock:
                self.require_control(session, body)
                self.require_motion_ready()
                if self.mode != "MANUAL":
                    raise Fault("invalid_state", "Select MANUAL mode")
                return await self.workers["motherboard"].call(op, body, job_id=self._new_job(session, request_id))
        if op == "camera.status":
            return await self.workers["camera"].call(op)
        if op == "localisation.status":
            return await self.workers["localisation"].call(op, body)
        if op in ("localisation.start", "localisation.stop"):
            async with self.capture_lock:
                self.require_control(session, body)
                self.require_game_idle()
                if op == "localisation.stop":
                    await self.workers["stream"].call("videostream.stop_source", {"name": "localisation"})
                    return await self.workers["localisation"].call(op)
                if self.mode not in ("MANUAL", "GAME"):
                    raise Fault("invalid_state", "Select MANUAL or start game")
                camera = await self.workers["camera"].call("camera.status")
                sync = camera.get("imu_sync", {})
                if not camera.get("running") or sync.get("state") != "synced":
                    raise Fault("not_ready", "A synchronized camera is required")
                return await self.workers["localisation"].call(op, {
                    "prior": body.get("prior"), "unicam_minus_stm": sync["unicam_minus_stm"],
                    "capture_id": next_id(),
                    "parameters": dict(self.params.values)}, timeout=5)
        if op == "detection.list":
            return {"detector": "colour_blobs", "profiles": list(COLOUR_DEFAULTS),
                    "max_blobs": 4, "coordinates": "image_pixels", "classifies_ball": False,
                    "modes": ['colour', 'ball', 'football'], "video_source": 'detection'}
        if op == "detection.status":
            return await self.workers["detection"].call(op)
        if op == 'detection.select':
            async with self.capture_lock:
                self.require_control(session, body)
                if set(body) - {'profile'}:
                    raise Fault('invalid_argument', 'Expected colour profile')
                return await self.workers['detection'].call(op, body)
        if op in ("detection.start", "detection.stop"):
            async with self.capture_lock:
                self.require_control(session, body)
                if op == "detection.start":
                    if set(body)-{'mode', 'profile'} or body.get('mode', 'colour') not in ('colour', 'ball', 'football'):
                        raise Fault('invalid_argument', 'Expected mode=colour, ball or football and optional colour profile')
                    if self.mode not in ("MANUAL", "GAME"):
                        raise Fault("invalid_state", "Select MANUAL or start game")
                    camera = await self.workers["camera"].call("camera.status")
                    self.require_control(session, body)
                    if self.mode not in ('MANUAL', 'GAME'):
                        raise Fault('invalid_state', 'Select MANUAL or start game')
                    if not camera['running']:
                        raise Fault("not_ready", "Start runtime camera before detection")
                    if body.get('mode') in ('ball', 'football'):
                        self.require_game_idle()
                        sync = camera.get('imu_sync', {})
                        if sync.get('state') != 'synced':
                            raise Fault('not_ready', 'A synchronized camera is required for ball diagnostics')
                        body = body | {'unicam_minus_stm': sync['unicam_minus_stm']}
                        if body.get('mode') == 'football':
                            stance = await self.workers['motherboard'].call('state')
                            if stance.get('active_job') or not stance.get('body_connected'):
                                raise Fault('not_ready', 'Football diagnostics require connected idle body; no motion is started')
                            body = body | {'pose': dict(active=True, pose_id=time.monotonic_ns(),
                                      pan=stance['head']['pan'], tilt=stance['head']['tilt'], after=time.monotonic(),
                                      note='Stationary football diagnostics; no motion')}
                else:
                    await self.workers['stream'].call('videostream.stop_source', {'name': 'detection'})
                result = await self.workers["detection"].call(op, body)
                self.football_diagnostic = op == 'detection.start' and body.get('mode') == 'football'
                return result
        if op in ("camera.start", "camera.stop"):
            if not _from_game:
                self.require_control(session, body)
            if 'with_imu' in body:
                raise Fault('invalid_argument', 'Runtime capture always requires IMU; use direct-gst for video without IMU')
            if set(body) - {'frame_duration_us', 'exposure_us', 'gain'}:
                raise Fault('invalid_argument', 'Unknown camera start setting; geometry is fixed')
            async with self.capture_lock:
                if not _from_game:
                    self.require_game_idle()
                    self.require_control(session, body)
                elif self.game.stop_requested or not self.game.info['running']:
                    raise Fault('cancelled', 'Game capture request cancelled')
                if op == "camera.stop":
                    return await self._stop_capture()
                if self.mode != "MANUAL" and not (_from_game and self.mode == 'GAME'):
                    raise Fault("invalid_state", "Select MANUAL mode")
                stream = await self.workers["stream"].call("state")
                if any(s["transport"] == "libcamera" for s in stream.get("active_streams", [])):
                    raise Fault("busy", "Stop direct-gst before starting capture")
                camera=await self.workers["camera"].call("camera.status")
                if camera.get('running') and camera.get('imu_sync',{}).get('state') in ('aligning','matched','synced'):
                    duration = number(body, 'frame_duration_us', camera['frame_duration_us'], 8333, 100000, True)
                    controls = camera.get('requested_controls', {})
                    requested = {'frame_duration_us': duration}
                    current = {'frame_duration_us': camera['frame_duration_us']}
                    for key, control, low, high, integer in (
                            ('exposure_us', 'exposure_us', 1, duration, True),
                            ('gain', 'gain', 1.0, 16.0, False)):
                        if key in body:
                            requested[key] = number(body, key, None, low, high, integer)
                            current[key] = controls.get(control)
                    if requested != current:
                        raise Fault('restart_required', 'Camera settings differ; stop capture explicitly before reconfiguration')
                    if not _from_game:
                        self.capture_explicit = True
                    return camera
                if camera["prepared"]:
                    raise Fault("busy", "Camera already prepared without requested IMU; stop capture explicitly to reconfigure")
                try:
                    result = await self.workers["camera"].call("camera.prepare", body, timeout=10)
                    self.capture_session = object()
                    self.alignment_pending.clear()
                    imu = await self.workers["motherboard"].call("imu.start", {
                        "frame_duration_us": result["frame_duration_us"]})
                    started = await self.workers["camera"].call("camera.start", {
                        "clock": imu["clock"]}, timeout=5)
                    self.capture_explicit = not _from_game
                    return started
                except Exception:
                    await self._stop_capture()
                    raise
        if op == 'camera.capabilities':
            return await self.workers['camera'].call(op)
        if op in ("videostream.list", "videostream.status", "videostream.capabilities",
                  "videostream.subscribe", "videostream.unsubscribe", "videostream.update", "videostream.stop"):
            async with self.capture_lock:
                return await self._video(session, op, body)
        raise Fault("not_supported", op)

    async def _discover_video_sources(self):
        # Explicit queries to configured IPC peers, never network discovery.
        for role, worker in self.workers.items():
            try:
                if not getattr(worker, "available", getattr(worker, "alive", True)):
                    raise Fault("worker_unavailable", role)
                reply = await worker.call("source.list")
                items = reply["items"]
                if len(items) > 1:
                    raise Fault("invalid_source", "Only one output per worker is supported")
                if items:
                    self.video_sources[role] = items[0] | {"name": role}
                elif role in self.video_sources:
                    self.video_sources[role].update(available=False, reason="output_removed", publishing=False)
            except (Fault, KeyError, TypeError):
                if role in self.video_sources:
                    self.video_sources[role].update(available=False, reason="worker_unavailable", publishing=False)
        stream = self.workers.get("stream")
        if stream is None or not getattr(stream, "available", getattr(stream, "alive", True)):
            return False
        try:
            await stream.call("sources.configure", {"items": list(self.video_sources.values())})
            await self._sync_video_sources()
            return True
        except Fault:
            return False

    async def _video_status(self, name, session, stream_alive=True):
        source = self.video_sources[name]
        if stream_alive:
            status = await self.workers["stream"].call("videostream.status", {
                "name": name, "operator_ip": session.id})
        else:
            status = {"name": name, "state": "unavailable", "settings": dict(source["settings"]),
                      "receivers": 0, "subscribed": False, "viewers": [], "error": "stream_worker_unavailable"}
        available, reason = source["available"], source["reason"]
        if not stream_alive:
            available, reason = False, "stream_worker_unavailable"
        elif source["transport"] == "libcamera" and status["state"] not in ("starting", "running"):
            camera = self.workers.get("camera")
            if camera is None or not getattr(camera, "available", getattr(camera, "alive", True)):
                available, reason = False, "camera_state_unknown"
            else:
                try:
                    state = await camera.call("camera.status")
                    if state.get("prepared"):
                        available, reason = False, "camera_busy"
                except Fault:
                    available, reason = False, "camera_state_unknown"
        for dependency in source["dependencies"]:
            other = self.workers.get(dependency)
            try:
                if other is None or not getattr(other, "available", getattr(other, "alive", True)) or not (await other.call(f"{dependency}.status")).get("running"):
                    available, reason = False, "dependency_not_ready"
            except Fault:
                available, reason = False, "dependency_not_ready"
        return status | {"available": bool(available), "reason": reason,
                         "producer": {"publishing": bool(source["publishing"]),
                                      "requested": bool(source["requested"])}}

    async def _video(self, session, op, body):
        if session.closed:
            raise Fault("session_expired", "Session closed")
        allowed = set()
        if op == "videostream.list":
            allowed |= {"offset", "limit"}
        elif op != "videostream.capabilities":
            allowed.add("name")
        if op in ("videostream.subscribe", "videostream.update"):
            allowed.add("settings")
        if op == "videostream.subscribe":
            allowed.add("rtp_port")
        if set(body) - allowed:
            raise Fault("invalid_argument", "Unknown videostream argument")
        stream_alive = await self._discover_video_sources()
        if op == "videostream.capabilities":
            return {"mtu": 1400, "max_receivers": 4,
                    "exact_osd": False, "rtcp": False}
        if op == "videostream.list":
            selected = page(list(self.video_sources), body, 1)
            items = []
            for name in selected["items"]:
                source = self.video_sources[name]
                status = await self._video_status(name, session, stream_alive)
                items.append({key: status[key] for key in
                              ("name", "state", "settings", "receivers", "subscribed", "viewers", "available", "reason", "producer")} |
                             {"title": source["title"], "controls": source["controls"]})
            return selected | {"items": items}
        name = body.get("name")
        if not isinstance(name, str) or name not in self.video_sources:
            raise Fault("not_found", "Unknown video output; query videostream.list")
        source = self.video_sources[name]
        status = await self._video_status(name, session, stream_alive)
        if op == "videostream.status":
            return status
        if not stream_alive:
            raise Fault("worker_unavailable", "Stream worker is unavailable", True)
        running = status["state"] in ("starting", "running")
        if op in ("videostream.update", "videostream.stop") or (op == "videostream.subscribe" and not running):
            self.require_control(session, body)
        if op in ("videostream.update", "videostream.subscribe"):
            from roki_ng.stream.sources import settings_for
            settings = settings_for(source, body.get("settings", {}), status["settings"])
            if op == "videostream.subscribe" and running and settings != status["settings"]:
                raise Fault("settings_conflict", "Output already running; subscribe without settings")
            if op == "videostream.update" and not body.get("settings"):
                raise Fault("invalid_argument", "Provide settings")
            if source["transport"] == "frames" and (running or op == "videostream.subscribe"):
                capture = self.workers.get("camera")
                if capture and (name == "camera" or "camera" in source["dependencies"]):
                    state = await capture.call("camera.status")
                    duration = state.get("frame_duration_us")
                    if duration and settings["max_fps"] > 1000000 / duration * 1.001:
                        raise Fault("invalid_argument", "max_fps exceeds capture FPS")
        args = body | {"operator_ip": session.id}
        if op == "videostream.subscribe":
            number(body, "rtp_port", None, 1024, 65535, True)
            args["host"] = session.address[0]
            if not running and not status["available"]:
                raise Fault("not_ready", status["reason"] or "Producer unavailable")
        try:
            # Recheck lease after IPC awaits; ownership may have changed meanwhile.
            if op in ("videostream.update", "videostream.stop") or (op == "videostream.subscribe" and not running):
                self.require_control(session, body)
            if session.closed:
                raise Fault("session_expired", "Session closed")
            if op == "videostream.subscribe" and not running and source["on_demand"]:
                await self.workers[name].call("source.start", {})
                source["requested"] = True
                self.require_control(session, body)
            await self.workers["stream"].call(op, args)
            if session.closed:
                await self.workers["stream"].call("videostream.detach_operator", {"operator_ip": session.id})
                raise Fault("session_expired", "Session closed during video operation")
        finally:
            await self._sync_video_sources()
        return await self._video_status(name, session)

    async def _sync_video_sources(self, force_off=False):
        stream = self.workers.get("stream")
        try:
            state = {} if force_off or stream is None or not getattr(stream, "alive", True) else await stream.call("state")
            wanted = {item["name"] for item in state.get("active_streams", [])}
        except Fault:
            wanted = set()
        for name, source in self.video_sources.items():
            if (source["on_demand"] and name not in wanted
                    and (source["requested"] or source["publishing"])):
                worker = self.workers.get(name)
                if worker and getattr(worker, "alive", True):
                    try:
                        await worker.call("source.stop", {})
                    except Fault as exc:
                        self.log("supervisor", "WARNING", f"Video output cleanup {name}: {exc}")
                    else:
                        source.update(requested=False, publishing=False)

    async def _video_fault(self, role, op):
        async with self.capture_lock:
            if role != "stream" and "stream" in self.workers:
                await self.workers["stream"].call("videostream.stop_source", {"name": role})
            await self._sync_video_sources(force_off=role == "stream" and op == "worker.fault")

    async def _reset_motherboard(self):
        if self.bootstrap is None:
            self.log('supervisor', 'ERROR', 'Automatic motherboard reset unavailable: no GPIO bootstrap owner')
            return
        attempt = 0
        async with self.capture_lock:
            try:
                await self._stop_capture()
            except Exception as exc:
                self.log('supervisor', 'WARNING', f'Capture stop before motherboard reset: {exc}')
            while not self.closing:
                attempt += 1
                self.log('supervisor', 'ERROR', f'Motherboard ACM lost: full STM GPIO reset, attempt {attempt}')
                try:
                    # Keep the GPIO thread alive until it releases reset; do not
                    # release its line underneath it during application shutdown.
                    boot = asyncio.create_task(asyncio.to_thread(self.bootstrap.start))
                    try:
                        await asyncio.shield(boot)
                    except asyncio.CancelledError:
                        await boot
                        raise
                    if self.closing:
                        return
                    await self.workers['motherboard'].call('motherboard.reconnect', timeout=10)
                    self.log('supervisor', 'INFO', 'STM reset complete; checking body, previous motion will not resume')
                    return
                except Exception as exc:
                    self.log('supervisor', 'ERROR', f'Motherboard reset failed: {exc}')
                    await asyncio.sleep(min(10, attempt * 2))

    async def _stop_capture(self):
        self.capture_session = None
        self.capture_explicit = False
        self.capture_unused_since = None
        self.alignment_pending.clear()
        # detection.stop is diagnostic-only; losing capture must also stop the
        # game-owned observer so the strategy can detect and recover the fault.
        try:
            await self.workers['detection'].call('ball.stop', timeout=5)
        except Fault as exc:
            self.log('supervisor', 'WARNING', f'Game observation stop: {exc}')
        if "localisation" in self.workers:
            try:
                await self.workers["localisation"].call("localisation.stop")
            except Fault as exc:
                self.log("supervisor", "WARNING", f"Localisation stop: {exc}")
        try:
            await self.workers["detection"].call("detection.stop")
            await self.workers['stream'].call('videostream.stop_source', {'name': 'detection'})
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Detection stop: {exc}")
        try:
            await self.workers["stream"].call("videostream.stop_source", {"name": "camera"})
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Runtime video stop: {exc}")
        try:
            return await self.workers["camera"].call("camera.stop", timeout=5)
        finally:
            await self.workers["motherboard"].call("imu.stop")

    async def _forward_alignment(self):
        while self.alignment_pending:
            session, body = self.alignment_pending.popleft()
            async with self.capture_lock:
                if session is not self.capture_session:
                    continue
                try:
                    state = await self.workers["camera"].call("camera.alignment", body)
                    if state["state"] == "matched":
                        await self.workers["motherboard"].call("imu.normal")
                        state = await self.workers["camera"].call("camera.sync_confirm")
                        self.alignment_pending.clear()
                        self.worker_event("camera", "camera.synchronized", state)
                except Exception as exc:
                    self.log("supervisor", "ERROR", f"Camera alignment failed: {exc}")
                    await self._stop_capture()

    async def _capture_fault(self, session):
        async with self.capture_lock:
            if session is not None and session is self.capture_session:
                await self._stop_capture()

    async def _set_parameters(self, values, expected=None):
        async with self.parameter_lock:
            if expected is not None and any(self.params.values.get(k)!=v for k,v in expected.items()):
                raise Fault('conflict','Parameter changed; reload before saving')
            values = {k: self.params.validate(k, v) for k, v in values.items()}
            self.require_game_parameters(values)
            validate_colour_ranges(self.params.values | values)
            applied = []
            try:
                for role, mode in (("motherboard", "next_job"), ("detection", "next_frame"),
                                   ("localisation", "next_frame"),
                                   ("camera", "next_request")):
                    update = {k: v for k, v in values.items() if SCHEMA[k][4] == mode}
                    if role in ('detection', 'localisation') and 'camera.custom_height_m' in values:
                        update['camera.custom_height_m'] = values['camera.custom_height_m']
                    if role=='localisation' and role not in self.workers:continue
                    if update:
                        previous = {k: self.params.values[k] for k in update}
                        try:
                            response=await self.workers[role].call("params.apply", update)
                        except Fault as exc:
                            if role != 'localisation' or exc.code != 'worker_unavailable':
                                raise
                            # Localisation receives the complete parameter snapshot on start.
                            self.log('supervisor', 'WARNING', 'Localisation unavailable; parameters will apply on its next start')
                            continue
                        # Camera.start may override persistent manual values for
                        # this capture. Roll back the actual controls, not defaults.
                        if role=='camera' and isinstance(response,dict) and set(response.get('previous',{}))==set(update):
                            previous=response['previous']
                        applied.append((role, previous))
                return await asyncio.to_thread(self.params.set_many, values)
            except Exception:
                for role, previous in reversed(applied):
                    await self.workers[role].call("params.apply", previous)
                raise

    async def _save_calibration(self, result):
        worker = self.workers["motherboard"]
        ident = result["job_id"]
        error = None
        async with self.parameter_lock:
            try:
                job = await worker.call("job.status", {"job_id": ident})
                values = result["values"]
                if (job["status"] not in ('saving', 'cancelled', 'failed') or job.get("calibration") != values
                        or set(values) != set(ROTATION_KEYS)):
                    raise Fault("calibration_invalid", "Unexpected calibration result")
                await asyncio.to_thread(self.params.set_many, values)
                self.log("supervisor", "INFO", f"Calibration {ident} saved: {values}")
            except Exception as exc:
                error = str(exc)
                self.log("supervisor", "ERROR", f"Calibration {ident} not saved: {error}")
            await worker.call("calibration.resolve", {"job_id": ident, "error": error})

    def _snapshot(self, topic):
        if topic == 'game.state':
            return self.game.state()
        if topic in BODY_TOPICS:
            return dict(self.body_telemetry.get(topic, {}))
        if topic == "system.workers":
            return {k: {"alive": w.alive, "state": w.state.get("state")} for k, w in self.workers.items()}
        role = {"motion.state": "motherboard", "camera.state": "camera", "videostream.state": "stream", "detection.state": "detection", "localisation.state": "localisation"}.get(topic)
        if role is None:
            raise Fault("not_found", "Unknown topic")
        worker = self.workers.get(role)
        return dict(worker.state) if worker else {"state": "starting"}

    def _sample(self, topic):
        if topic == 'game.state':
            return {'topic': topic, 'valid': True, 'source_mono_ns': time.monotonic_ns(),
                    'age_ms': 0, 'data': self.game.state()}
        if topic in BODY_TOPICS:
            data = self._snapshot(topic)
            if topic == 'body.imu':
                roll, pitch, yaw = (data.pop(key, None) for key in ('roll_deg', 'pitch_deg', 'yaw_rad'))
                data['rpy_rad'] = [math.radians(roll) if roll is not None else None,
                                   math.radians(pitch) if pitch is not None else None, yaw]
            source = data.get("source_mono_ns")
            age = (time.monotonic_ns() - source) / 1e6 if source is not None else None
            worker = self.workers.get("motherboard")
            limit = BODY_TOPIC_MAX_AGE_MS[topic]
            valid = bool(worker and worker.alive and data.get("valid")
                         and age is not None and 0 <= age < limit)
            return {"topic": topic, "valid": valid, "source_mono_ns": source,
                    "age_ms": round(age) if age is not None else None,
                    "data": data | {"valid": valid}}
        role = {"motion.state": "motherboard", "camera.state": "camera", "videostream.state": "stream", "detection.state": "detection", "localisation.state": "localisation"}.get(topic)
        worker = self.workers.get(role)
        observed = worker.last_heartbeat if worker else time.monotonic()
        if topic == "localisation.state":
            data = self._snapshot(topic)
            age = data.get('age_ms')
            transport_age = max(0, round((time.monotonic()-observed)*1000))
            valid = bool(worker and worker.alive and data.get('running') and not data.get('error')
                         and (data.get('result') or {}).get('valid')
                         and type(age) in (int,float) and 0<=age+transport_age<=500)
            return {"topic": topic, "valid": valid,
                    "source_mono_ns": int(observed * 1e9),
                    "age_ms": transport_age, "data": data}
        return {"topic": topic, "valid": bool(worker.alive) if worker else topic == "system.workers",
                "source_mono_ns": int(observed * 1e9),
                "age_ms": round((time.monotonic() - observed) * 1000), "data": self._snapshot(topic)}

    async def _sync_body_watch(self):
        async with self.body_watch_lock:
            wanted = any(not s.closed and any(t in s.data for t in BODY_TOPICS)
                         for s in self.sessions.values())
            servos = any(not s.closed and "body.servos" in s.data for s in self.sessions.values())
            power = any(not s.closed and "body.power" in s.data for s in self.sessions.values())
            imu = any(not s.closed and any(t in s.data for t in BODY_TOPICS if t != "body.power")
                      for s in self.sessions.values())
            if (wanted != self.body_watch or servos != self.body_servo_watch
                    or power != self.body_power_watch or imu != self.body_imu_watch):
                args = {"enabled": wanted, "imu": imu, "power": power}
                if servos or self.body_servo_watch:
                    args["servos"] = servos
                await self.workers["motherboard"].call("body.telemetry.watch", args)
                self.body_watch = wanted
                self.body_servo_watch = servos
                self.body_power_watch = power
                self.body_imu_watch = imu

    async def _data(self, session, op, body):
        if op == "data.list":
            return {"items": [{"name": t, "kind": "state",
                               "max_rate_hz": TOPIC_MAX_RATE_HZ.get(t, 10)} for t in TOPICS]}
        if op == "data.snapshot":
            if body.get("topic") in BODY_TOPICS:
                flag = {"body.servos": "servos", "body.power": "power"}.get(body["topic"])
                args = {flag: True} if flag else {}
                self.body_telemetry = await self.workers["motherboard"].call("body.telemetry.read", args)
            return self._sample(body.get("topic"))
        if op in ("data.subscribe", "data.update"):
            topic = body.get("topic")
            self._snapshot(topic)
            max_rate = TOPIC_MAX_RATE_HZ.get(topic, 10)
            rate = number(body, "rate_hz", min(2, max_rate), 0.2, max_rate)
            session.data[topic] = {"rate": rate, "next": 0, "sequence": 0}
            if topic in BODY_TOPICS:
                try:
                    await self._sync_body_watch()
                except Exception:
                    session.data.pop(topic, None)
                    raise
            return {"subscription_id": topic, "rate_hz": rate}
        if op == "data.unsubscribe":
            session.data.pop(body.get("subscription_id"), None)
            await self._sync_body_watch()
            return {}
        raise Fault("not_supported", op)

    def _logs(self, session, op, body):
        if op == "log.sources":
            return page(sorted({r["source"] for r in self.history}), body)
        if op == "log.unsubscribe":
            session.logs = None
            return {}
        level = body.get("level", "INFO")
        sources = body.get("sources", [])
        if level not in LEVELS or not isinstance(sources, list) or not all(isinstance(s, str) for s in sources):
            raise Fault("invalid_argument", "Invalid log filter")
        after = number(body, "after", self.log_id if op != "log.snapshot" else 0, 0, 2**64 - 1, True)
        if op in ("log.subscribe", "log.update"):
            session.logs = {"level": level, "sources": sources, "after": after}
            return {"subscription_id": "logs", "after": after}
        records = [r for r in self.history if r["record_sequence"] > after and self._log_matches(r, level, sources)]
        limit = number(body, "limit", 2, 1, 2, True)
        return {"records": records[:limit], "next_after": records[min(limit, len(records))-1]["record_sequence"] if records else after,
                "oldest": self.history[0]["record_sequence"] if self.history else 0}

    @staticmethod
    def _log_matches(record, level, sources):
        return LEVELS.get(record["level"], 20) >= LEVELS[level] and (not sources or record["source"] in sources)

    async def _release(self, session):
        async with self.control_lock:
            if self.owner != session.id:
                return
            if self.game.info['running'] and self.mode == 'GAME':
                # The operator lease is not the autonomous body's lifetime.
                self.owner = None
                session.latest_drive = None
                return
            await self.game.stop('control_released', hard=self.mode == 'FAULT')
            self.owner = None
            session.latest_drive = None
            self.menu_motion = False
            try:
                await self.workers["motherboard"].call("control.release", urgent=True)
            except Fault as exc:
                self.log("supervisor", "ERROR", f"Control release: {exc}")
            if self.mode != "FAULT":
                self.mode = "IDLE"

    async def _expire(self, session):
        if session.closed:
            return
        session.closed = True
        await self._release(session)
        try:
            await self._sync_body_watch()
        except Fault as exc:
            self.log("supervisor", "WARNING", f"Body telemetry release: {exc}")
        async with self.capture_lock:
            try:
                if 'stream' in self.workers:
                    await self.workers["stream"].call("videostream.detach_operator", {"operator_ip": session.id})
            except Fault as exc:
                self.log('stream', 'WARNING', f'Detach expired session: {exc}')
            await self._sync_video_sources()
        self.sessions.pop(session.id, None)
        for ident in session.jobs.values():
            self.job_routes.pop(ident, None)

    async def _media_lifecycle(self):
        while not self.closing:
            await asyncio.sleep(1)
            try:
                async with self.capture_lock:
                    await self._retire_media()
            except Fault as exc:
                self.log('supervisor', 'WARNING', f'Media lifecycle: {exc}')

    async def _retire_media(self, *, grace=5., now=None):
        """Called under capture_lock; queries and retirement never start workers."""
        if self.closing:
            return
        now = time.monotonic() if now is None else now
        game_active = self.game.starting or self.game.info['running']
        # Capture started by the game is not an explicit operator camera start.
        # Release it only after the game and all other consumers have gone away.
        camera = self.workers['camera']
        if camera.alive and self.capture_session is not None and not self.capture_explicit and not game_active:
            detection = await self.workers['detection'].call('detection.status')
            ball = await self.workers['detection'].call('ball.status')
            localisation = await self.workers['localisation'].call('localisation.status')
            streams = await self.workers['stream'].call('state')
            used = (detection.get('running') or ball.get('running') or localisation.get('running')
                    or any(s['name'] == 'camera' or 'camera' in s['dependencies']
                           for s in streams.get('active_streams', [])))
            if used:
                self.capture_unused_since = None
            else:
                if self.capture_unused_since is None:
                    self.capture_unused_since = now
                if now - self.capture_unused_since >= grace:
                    await self._stop_capture()
        else:
            self.capture_unused_since = None
        for role in ('stream', 'detection', 'localisation', 'camera'):
            worker = self.workers[role]
            if isinstance(worker, LazyWorkerPeer):
                await worker.retire_if_idle(held=game_active and role != 'stream', grace=grace, now=now)

    async def _maintenance(self):
        while True:
            await asyncio.sleep(0.05)
            now = time.monotonic()
            for session in list(self.sessions.values()):
                if now - session.seen > 2 and not session.closed:
                    self.spawn(self._expire(session))
                    continue
                for topic, sub in session.data.items():
                    if now >= sub["next"]:
                        sub["next"] = now + 1 / sub["rate"]
                        sub["sequence"] += 1
                        self.send(session, "sample", "data.sample", {"subscription": topic,
                                  "sequence": sub["sequence"], **self._sample(topic)})
                if session.logs:
                    sub = session.logs
                    records = [r for r in self.history if r["record_sequence"] > sub["after"]]
                    dropped = max(0, self.history[0]["record_sequence"] - sub["after"] - 1) if self.history else 0
                    selected = []
                    for record in records:
                        sub["after"] = record["record_sequence"]
                        if self._log_matches(record, sub["level"], sub["sources"]):
                            selected.append(record)
                            if len(selected) == 2:
                                break
                    if selected or dropped:
                        self.send(session, "sample", "log.sample", {"subscription": "logs", "records": selected, "dropped": dropped})
            for worker in list(self.workers.values()):
                if (worker is self.workers.get('motherboard') and self.motherboard_reset_task
                        and not self.motherboard_reset_task.done()):
                    continue  # Bounded ACM reopen may outlast a normal heartbeat.
                if worker.alive and now - worker.last_heartbeat > 5:
                    worker._failed("Heartbeat timeout; process will be terminated")
                    if not self._game_media_recovery(worker.role):
                        self.spawn(worker.close())
            if self.head_menu:
                await self.head_menu.observe_game(self.game.state())

    async def close(self):
        self.closing = True
        await self.game.stop('shutdown', hard=True)
        if self.button_task:
            self.button_task.cancel()
            await asyncio.gather(self.button_task, return_exceptions=True)
        if self.head_buttons:
            self.head_buttons.close()
        if self.head_menu:
            await self.head_menu.close()
        if self.voice:
            await self.voice.close()
        for worker in reversed(list(self.workers.values())):
            await worker.close()
        for task in list(self.tasks):
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        if self.sock:
            self.sock.close()
        if self.bootstrap:
            self.bootstrap.close()
