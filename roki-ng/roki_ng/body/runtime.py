"""Single ACM owner and cancellable, bounded manual motion jobs."""

import collections
import copy
import json
import math
import struct
from pathlib import Path
from types import SimpleNamespace
import time
from roki_ng.identifiers import next_id

from roki_ng.body.motion.backends import load
from roki_ng.wire import Fault
from roki_ng.wire import boolean
from roki_ng.wire import choice
from roki_ng.wire import number
from roki_ng.wire import page
from roki_ng.body.calibration import TestPlan
from roki_ng.body.calibration import TITLES
from roki_ng.body.calibration import MEASUREMENTS
from roki_ng.body.calibration import describe
from roki_ng.body.calibration import validate_args
from roki_ng.body.calibration import wrap
import roki_ng.body.motion.slots as slots
from roki_ng.body.imu_fallback import FallbackImu
from roki_ng.body.body_imu import BodyImu
from roki_ng.body.body_imu import quaternion_yaw
from roki_ng.body.servos import BodyServos
from roki_ng.body.power import BodyPower
from roki_ng.body.stabilization import CrouchStabilizer
from roki_ng.body.imu_status import ImuDisplayStatus
from roki_ng.body.step_adaptation import StepAdaptation
from roki_ng.body.stabilization_diagnostics import StabilizationDiagnostics
from roki_ng.body.controller_kick import KICK_SLOT
from roki_ng.body.controller_kick import STOP_SLOT
from roki_ng.body.controller_kick import KICK_PARAMETERS
from roki_ng.body.controller_kick import KickWitness


from roki_ng.body.communication.transports.motherboard import BodyTransmissionError, RokiHardware, SimHardware


class Body:
    def __init__(self, config, emit, log):
        self.emit, self.log = emit, log
        self.simulated = config.get("simulate", False)
        self.parameters = config["parameters"]
        self.backend = load(config.get('robot_model'))
        self._check_model_parameters(self.parameters)
        self.hardware = SimHardware(config) if self.simulated else RokiHardware(config)
        self.body_connected = self.simulated
        self.body_disabled = config.get("body_disabled", False)
        if self.body_disabled:
            self.body_connected = False
        self.imu = None
        self.imu_error = None
        self.imu_published = 0
        self.imu_rpc_errors = 0
        self.reconnect_at = 0
        self.motherboard_reset_pending = False
        self.retry_delay = 1.0
        self.probe_successes = 0
        self.important_errors = collections.deque(maxlen=3)
        self.last_important_error = None
        self.queue_reset_pending = False
        self.queue_reset_at = 0
        self.post_kick_walk = False
        self.protect_head_until = 0.0
        self.previous_pitch = None
        self.fall_sequence = 0
        self.fall_samples = 0
        self.recovery_active = False
        self.model = self.backend.Robot()
        self.hip_limits = {(servo, bus): sorted((7500 + low * sign, 7500 + high * sign))
                           for servo, bus, sign, _, low, high, *_ in self.model.ACTIVESERVOS
                           if servo == 7}
        self.engine = None
        self.servo_targets = {}
        self.sent_targets = {}
        self.target_times = {}
        self.body_servos = BodyServos(self.model)
        self.servo_watch = False
        self.body_imu = BodyImu()
        self.head_settled_at = float('inf')
        calibration_path = Path('/var/lib/roki-ng/head-body-imu.json')
        if not self.simulated and calibration_path.exists():
            calibration = json.loads(calibration_path.read_text())
            if calibration.get('enabled', False):
                self.body_imu = FallbackImu(calibration)
        self.imu_display_status = None if self.simulated else ImuDisplayStatus()
        self.step_adaptation = None
        self.calibration_walking = False
        self.body_power = BodyPower(self.simulated)
        self.power_watch = False
        self.imu_watch = False
        self.stabilizer = CrouchStabilizer()
        self.stabilization_diagnostics = StabilizationDiagnostics()
        self.telemetry_watch = False
        self.telemetry_at = 0
        self.stabilization_warning = None
        self.stabilization_log_at = 0
        self.pose = "unknown"
        self.control = False
        self.error = None
        self.active = None
        self.plan = None
        self.jobs = collections.OrderedDict()
        self.calibration_pending = None
        self.drive = None
        self.drive_rearm_required = False
        self.drive_deadline = 0
        self.stop_requested = False
        self.next_at = 0
        self.instruction = None
        self.segment_active = False
        self.instruction_started = 0
        self.head = {"pan": 0, "tilt": 0}
        self.slots = sorted(p.stem for p in (self.backend.ASSETS / "slots").glob("*.json"))
        self.jumps = json.loads((self.backend.ASSETS / "jumps.json").read_text())
        self.log("INFO", "Motherboard ready; body connection checked separately; camera remains off")

    def state(self):
        body_imu = self.body_imu.state(time.monotonic())
        return {"state": "degraded" if not self.body_connected else "fault" if self.error else "ready", "pose": self.pose,
                "body_connected": self.body_connected,
                "recovering": self.recovery_active,
                "body_disabled": self.body_disabled, "imu_running": self.imu is not None,
                "imu_published": self.imu.published if self.imu else self.imu_published,
                "imu_invalid": self.imu.lost if self.imu else 0,
                "imu_error": self.imu_error,
                "imu_rpc_errors": self.imu.total_errors if self.imu else self.imu_rpc_errors,
                "active_job": self.active, "head": self.head, "error": self.error,
                "important_errors_5s": self._important_error_count(),
                "last_important_error": self.last_important_error,
                "queue_reset_pending": self.queue_reset_pending,
                "body_imu": {key: body_imu[key] for key in ("valid", "sequence", "source_mono_ns")},
                "simulated": self.simulated}

    def _engine(self):
        if self.engine is None:
            from roki_ng.body.motion.engine import walking_engine
            self.engine = walking_engine(self.parameters, log=self.log, backend=self.backend)
        return self.engine

    def command(self, op, args):
        # Nested commands (takeover -> stop) must not count the same failed
        # transaction twice. Each top-level call is a distinct attempt.
        nested = getattr(self, "in_command", False)
        self.in_command = True
        try:
            return self._command(op, args)
        except (Fault, OSError) as exc:
            if op.startswith("imu."):
                self.imu_error = str(exc)[:240]
                self.log("WARNING", f"{op} failed: {exc}")
            elif not nested and (isinstance(exc, OSError) or exc.code == "hardware_error"):
                self._important_failure(exc)
            elif not nested:
                self.log("WARNING", f"{op} rejected: {exc}")
            raise
        finally:
            self.in_command = nested

    def _command(self, op, args):
        if op == 'motherboard.reconnect':
            if not self.motherboard_reset_pending:
                raise Fault('invalid_state', 'No motherboard reset pending')
            self.hardware.reconnect_motherboard()
            self.motherboard_reset_pending = False
            self.probe_successes = 0
            self.reconnect_at = 0
            return self.state()
        if op == "body.telemetry.watch":
            self.telemetry_watch = boolean(args, "enabled", False)
            self.servo_watch = self.telemetry_watch and boolean(args, "servos", False)
            self.imu_watch = self.telemetry_watch and boolean(args, "imu", True)
            self.power_watch = self.telemetry_watch and boolean(args, "power", False)
            self.telemetry_at = 0
            return {}
        if op == "body.telemetry.read":
            power, servos = boolean(args, "power", False), boolean(args, "servos", False)
            if self.body_connected:
                try:
                    if power:
                        self.body_power.read(self.hardware)
                    elif servos:
                        self.body_servos.read(self)
                    # IMU snapshots never initiate a hardware transaction.
                except (Fault, OSError) as exc:
                    self.log("WARNING", f"Body telemetry read failed: {exc}")
            return self._body_telemetry()
        if op == "control.takeover":
            self.control = False
            self.command("motion.stop_hard", {})
            self.control = True
            return {"stop_confirmed": True}
        if op == "imu.stop":
            if self.imu:
                self.imu_published = self.imu.published
                self.imu_rpc_errors = self.imu.total_errors
                current, self.imu = self.imu, None
                current.close()
            elif not self.simulated:
                self.hardware.check(self.hardware.mb.StopStrobeCapture(), self.hardware.mb)
            return self.state()
        if op == "imu.normal":
            if not self.imu:
                raise Fault("not_ready", "IMU stream not running")
            return self.imu.normal()
        if op == "imu.start":
            if self.imu:
                raise Fault("busy", "IMU already running")
            if self.simulated:
                raise Fault("not_supported", "IMU requires STM")
            from roki_ng.motherboard.imu import ImuPublisher
            duration = number(args, "frame_duration_us", 16667, 8333, 100000, True)
            candidate = ImuPublisher(self.hardware, self.emit)
            try:
                clock = candidate.start(duration)
            except Exception:
                candidate.close()
                raise
            self.imu, self.imu_error = candidate, None
            return self.state() | {"clock": clock}
        if op == "calibration.resolve":
            ident = args.get("job_id")
            if ident != self.calibration_pending or ident not in self.jobs:
                raise Fault("not_found", "No matching calibration pending")
            job = self.jobs[ident]
            error = args.get("error")
            if error is None:
                self.parameters.update(job["calibration"])
                self.engine = None
            job['saved'] = error is None
            self.calibration_pending = None
            if self.active == ident:
                if error:
                    self._finish('failed', error)
                else:
                    job.update(status='running', stage='calibration_saved', reason=None)
                    self.emit('job.progress', dict(job))
            else:
                # Cancellation/fall during disk I/O must never restart the plan.
                if job['status'] == 'saving':
                    job.update(status='failed' if error else 'completed', reason=error)
                self.emit('job.failed' if job['status'] == 'failed' else 'job.completed', dict(job))
            return dict(job)
        if self.calibration_pending and op in (
                "control.acquire", "params.apply", "test.start", "motion.pose", "motion.drive",
                "motion.jump", "motion.slot", "motion.kick", "motion.head",
                "motion.get_up", "motion.splits", "game.walk", "game.jump", "game.head"):
            raise Fault("busy", "Calibration parameters are being saved", True)
        if op == "control.acquire":
            if self.active:
                raise Fault("busy", "Previous motion is still stopping", True)
            self.control = True
            return {}
        if op == "control.release":
            self.control = False
            self.drive = None
            if self.recovery_active:
                self.command("motion.stop_hard", {})
            elif self.active and self.jobs[self.active]["operation"] in ("motion.drive", "test.start"):
                self.stop_requested = True
            else:
                self.command("motion.stop_hard", {})
            return {}
        if op == "state":
            return self.state()
        if op == "params.apply":
            self._check_model_parameters(self.parameters | args)
            if self.active:
                raise Fault("busy", "Stop motion before applying motion parameters", True)
            geometry_changed = any(key in args and args[key] != self.parameters[key]
                                   for key in ("walk.gait_height_mm", "walk.sway_amplitude_mm", "walk.roki1_pult_enabled"))
            self.parameters.update(args)
            if self.engine:
                self.engine.configure(self.parameters)
            if geometry_changed:
                self.engine = None
                if self.pose in ("crouch", "crouch_centered"):
                    self.pose = "unknown"
            return {}
        if op == "motion.slots":
            return page(self.slots, args)
        if op == "motion.joints":
            return page(self.body_servos.catalog, args)
        if op == "job.status":
            job = self.jobs.get(args.get("job_id"))
            if job is None:
                raise Fault("not_found", "Job no longer retained")
            if job['operation'] == 'game.walk' or (job['operation'] == 'game.head' and self.pose == 'crouch'):
                from roki_ng.body.motion.game_geometry import camera_height
                from roki_ng.vision.camera_geometry import height as optical_height
                height = optical_height(self.parameters, self.model.camera)
                if (self.active == job['job_id'] or self.pose == 'crouch') and self.body_imu.fresh(time.monotonic()):
                    height = camera_height(self.parameters, self.model, self.engine,
                                           self.body_imu.pitch, self.body_imu.roll,
                                           self.head['pan'], self.head['tilt'])
                return job | {'camera_geometry': {'height_m': height, 'stamp': time.monotonic()}}
            return job
        if op == "motion.stop_hard":
            controller_stop = self.hardware.controller_pending
            if controller_stop and self.active:
                self.jobs[self.active]['execution_uncertain'] = True
            self.drive = None
            self.plan = self.instruction = None
            self.pose = "unknown"
            self.engine = None
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate("hard_stop")
            self.stabilizer.reset("hard_stop")
            try:
                self.hardware.reset()
            except Exception as exc:
                self.error = None if isinstance(exc, Fault) and exc.code == 'body_busy' else str(exc)
                self.queue_reset_pending = True
                self.queue_reset_at = time.monotonic() + .05
                self._finish("failed", str(exc))
                raise
            self._finish("cancelled", "hard_stop")
            if self.body_connected:
                self.error = None
            return {"stopped": True, "pose": "unknown", "scope": "host_body_queue_and_controller" if controller_stop else "host_body_queue",
                    "body_connected": self.body_connected}
        if op in ("motion.stop_graceful", "job.cancel"):
            if op == "job.cancel" and args.get("job_id") != self.active:
                raise Fault("not_found", "Job is not active")
            if self.recovery_active:
                return self.command("motion.stop_hard", {})
            if self.active and self.jobs[self.active].get('kick_type') == 'controller':
                self.stop_requested = True
                return {"accepted": True, "job_id": self.active}
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start", "game.walk", "game.jump"):
                raise Fault("not_supported", "This motion requires hard stop or completion")
            self.stop_requested = True
            self.drive = None
            return {"accepted": True, "job_id": self.active}
        if op == "test.list":
            return {"items": list(TITLES)}
        if op == "test.describe":
            validated, _ = validate_args(args)
            return describe(validated['name'], validated)
        if not self.control:
            raise Fault("not_owner", "Motion control has been released")
        if not self.body_connected:
            raise Fault("body_unavailable", "Body disconnected; reconnect pending", True)
        if self.queue_reset_pending:
            raise Fault("body_unavailable", "Body queue reset not confirmed", True)
        if self.error:
            raise Fault("motion_fault", self.error)
        if op == "motion.head":
            if self.recovery_active:
                raise Fault("busy", "Get-up sequence owns the head", True)
            if self.active and self.jobs[self.active]["operation"] not in ("motion.drive", "test.start"):
                raise Fault("busy", "Software slot/pose owns the head", True)
            pan = number(args, "pan", self.head["pan"], -2666, 2666, True)
            tilt = number(args, "tilt", self.head["tilt"], -2600, 950, True)
            frames = number(args, "frames", 10, 1, 100, True)
            self.hardware.prepare_motion()
            values = [SimpleNamespace(Id=0, Sio=1, Data=7500 + pan),
                      SimpleNamespace(Id=12, Sio=2, Data=7500 + tilt)]
            self.head_settled_at = time.monotonic() + frames * .02 + .4
            self.hardware.head(values, frames)
            self.servo_targets.update(((v.Id, v.Sio), v.Data) for v in values)
            self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in values)
            self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in values)
            self.head = {"pan": pan, "tilt": tilt}
            return {"accepted": True, "target": dict(self.head)}
        if op == "motion.drive":
            drive = {axis: number(args, axis, 0.0, -1, 1) for axis in ("x", "y", "yaw")}
            drive["speed"] = number(args, "speed", 0.5, 0.1, 1)
            drive["crouch"] = choice(args, "crouch", "off", ("off", "on", "centered"))
            drive["heading_hold"] = boolean(args, "heading_hold", False)
            active = any(abs(drive[a]) > 0.05 for a in ("x", "y", "yaw"))
            if not active:
                self.drive_rearm_required = False
            elif self.drive_rearm_required:
                raise Fault('not_ready', 'Release walking controls before restarting after a fall or lost IMU')
            if self.active and self.jobs[self.active]["operation"] != "motion.drive":
                raise Fault("busy", "One-shot motion running", True)
            if self.stop_requested and self.active:
                raise Fault("busy", "Finishing current walk; wait for completion", True)
            self.drive, self.drive_deadline = drive, time.monotonic() + 0.7
            if active and not self.active:
                return self._start(op, self._walk())
            return {"accepted": True, "job_id": self.active}
        if op == 'game.observe':
            if not self.active or args.get('job_id') != self.active or self.jobs[self.active]['operation'] != 'game.walk':
                return {'accepted': False}  # Feedback may race job completion.
            stamp = number(args, 'stamp', None, 0, 1e12)
            if not 0 <= time.monotonic()-stamp <= .5:
                return {'accepted': False}
            ball = (number(args, 'x_m', None, -4, 4), number(args, 'y_m', None, -4, 4))
            if stamp > getattr(self, 'game_ball_stamp', 0):
                self.game_ball, self.game_ball_stamp = ball, stamp
            return {'accepted': True}
        if self.active:
            raise Fault("busy", "Motion already running; command discarded", True)
        self.football_arms = (self.backend.NAME == 'roki2' and
                              op in ('motion.pose', 'motion.kick', 'game.head', 'game.walk', 'game.jump')
                              and boolean(args, 'football_arms', False))
        if op == 'game.walk':
            keep_crouch = boolean(args, 'keep_crouch', False)
            if self.pose != 'base_stand' and not (keep_crouch and self.pose == 'crouch'):
                raise Fault('invalid_state', 'Game walking requires a prepared stance')
            kind = choice(args, 'kind', None, ('omni', 'approach', 'around', 'verify', 'kickoff', 'kickoff_arc'))
            options = {key: number(args, key, default, low, high) for key, default, low, high in (
                ('distance_m', 0., -4., 4.), ('course', 0., -math.pi, math.pi),
                ('heading', 0., -math.pi, math.pi), ('initial_heading', 0., -math.pi, math.pi),
                ('yaw_offset', 0., -math.pi, math.pi), ('kick_heading', 0., -math.pi, math.pi),
                ('x_m', 0., -4., 4.), ('y_m', 0., -4., 4.))}
            options['trim_cycles'] = number(args, 'trim_cycles', 0, 0, 2, True)
            options['soft_touch'] = boolean(args, 'soft_touch', False)
            options['keep_crouch'] = keep_crouch
            options['live_approach'] = boolean(args, 'live_approach', False)
            if 'stop_x_m' in args:
                options['stop_x_m'] = number(args, 'stop_x_m', None, .08, .5)
            if 'target_y_m' in args:
                options['target_y_m'] = number(args, 'target_y_m', None, -.5, .5)
            if options['live_approach'] and (kind != 'approach' or 'stop_x_m' not in options or 'target_y_m' not in options):
                raise Fault('invalid_argument', 'Live approach requires both foot target coordinates')
            options['invert'] = number(args, 'invert', 1, -1, 1, True)
            if options['invert'] not in (-1, 1):
                raise Fault('invalid_argument', 'Kickoff direction must be -1 or 1')
            self.game_ball = (options['x_m'], options['y_m'])
            self.game_ball_stamp = number(args, 'stamp', 0., 0., 1e12)
            return self._start(op, self._game_walk(kind, options), {'segments': [], 'execution_uncertain': False})
        if op == "game.jump":
            direction = choice(args, "direction", None, tuple(self.jumps) + ("turn_left", "turn_right"))
            fraction = number(args, "fraction", 1., .1, 1.)
            return self._start(op, self._game_jump(direction, fraction), {"segments": [], "execution_uncertain": False})
        if op == "game.head":
            pan = number(args, 'pan', 0, -2666, 2666, True)
            tilt = number(args, 'tilt', None, -2600, 950, True)
            return self._start(op, self._head(pan, tilt))
        if op == "motion.pose":
            name = choice(args, "name", None, ("base_stand", "crouch", "stand", "head_field"))
            if name == "crouch" and choice(args, "crouch", "on", ("on", "centered")) == "centered":
                name = "crouch_centered"
            return self._start(op, self._pose(name))
        if op == "motion.slot":
            name = args.get("name")
            if name not in self.slots:
                raise Fault("not_found", "Unknown software slot")
            factor = number(args, "speed_factor", 1.0, 0.25, 2)
            document = json.loads((self.backend.ASSETS / "slots" / f"{name}.json").read_text())
            if "frames" in document:
                steps = slots.decode(document, self.model, factor)
                return self._start(op, self._individual_slot(steps))
            if document.get("units", "kondo") != "kondo":
                raise Fault("invalid_motion", "Legacy combined-knee rows require Kondo units")
            rows = document[name]
            self._validate_rows(rows, factor)
            return self._start(op, self._slot(rows, factor))
        if op == "motion.jump":
            direction = choice(args, "direction", "forward", tuple(self.jumps) + ("turn_left", "turn_right"))
            fraction = number(args, "fraction", 1.0, 0.1, 1)
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            if direction.startswith('turn_') and self.parameters.get('walk.roki1_pult_enabled', False):
                return self._start(op, self._pult_turn(direction, fraction, crouch))
            if crouch != "off":
                target = "crouch_centered" if crouch == "centered" else "crouch"
                if self.pose == target or self.pose == "base_stand":
                    steps = self._relative_jump_steps(direction, fraction)
                    return self._start(op, self._relative_jump(steps))
                return self._start(op, self._prepare_jump(direction, fraction, target))
            rows = self._jump_rows(direction, fraction)
            return self._start(op, self._slot(rows, legs_only=direction.startswith("turn_")))
        if op == "motion.get_up":
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            return self._start(op, self._get_up(crouch))
        if op == "motion.splits":
            kind = choice(args, "kind", "small", ("small", "big"))
            self.backend.splits(kind == 'big')  # Reject unavailable primitives before preparing hardware.
            crouch = choice(args, "crouch", "off", ("off", "on", "centered"))
            return self._start(op, self._splits(kind, crouch))
        if op == "motion.kick":
            leg = choice(args, "leg", "right", ("right", "left"))
            soft = boolean(args, 'soft', False)
            kick_type = 'software' if soft or self.football_arms else self.parameters['motion.kick_type']
            self._check_model_parameters(self.parameters)
            power = number(args, "power", 80, 20 if kick_type == 'controller' else 1, 100, True)
            offset = number(args, "offset", 0, -60, 80, True)
            return self._start(op, self._kick(leg, power, offset, kick_type, soft=soft),
                               {'kick_type': kick_type, 'execution_uncertain': False})
        if op == "test.start":
            plan = TestPlan(self, args)
            return self._start(op, plan.run(), {"test": plan.description,
                               "manual_parameters": list(MEASUREMENTS.get(plan.name, ())),
                               "measurement_baseline": {key: plan.parameters[key]
                                   for key in MEASUREMENTS["spot_run"]} if plan.name == "spot_run" else {}})
        raise Fault("not_supported", op)

    def _check_model_parameters(self, values):
        if values.get('walk.roki1_pult_enabled'):
            if self.backend.NAME != 'roki1':
                raise Fault('unsupported', 'Experimental walk_pult requires Roki1')
            if values['motion.frame_ms'] != 20:
                raise Fault('invalid_argument', 'Experimental walk_pult requires motion.frame_ms=20')
        if not self.backend.CONTROLLER_KICK and values.get('motion.kick_type') == 'controller':
            raise Fault('unsupported', 'Controller kick contract is not verified for this robot model')
        if not self.backend.STABILIZATION and values.get('stabilization.enabled'):
            raise Fault('unsupported', 'Crouch stabilization is not validated for this robot model')

    def _start(self, op, plan, details=None):
        self.hardware.prepare_motion()
        if op == "test.start":
            self.engine = None
            self.pose = "unknown"
        if op in ('test.start', 'motion.drive', 'motion.kick') or op.startswith('game.'):
            self.previous_pitch = None
            self.fall_samples = 0
            self.fall_sequence = self.body_imu.sequence
        ident = getattr(self, 'request_job_id', None) or -next_id()
        if getattr(self, 'football_arms', False) and op in (
                'motion.pose', 'motion.kick', 'game.head', 'game.walk', 'game.jump'):
            if op != "game.head":
                self.jump_arms_lowered = op == "game.jump"
            plan = self._guard_football_arms(plan, lowered=getattr(self, "jump_arms_lowered", False))
        self.active, self.plan = ident, iter(plan)
        self.segment_active = False
        self.stop_requested = False
        self.next_at, self.instruction = 0, None
        self.jobs[ident] = {"job_id": ident, "operation": op, "status": "running", "progress": 0}
        self.jobs[ident].update(details or {})
        while len(self.jobs) > 64:
            self.jobs.popitem(last=False)
        self.emit("job.progress", dict(self.jobs[ident]))
        result = {"accepted": True, "job_id": ident}
        if op == 'test.start' and 'test' in self.jobs[ident]:
            result.update(test=self.jobs[ident]['test'], progress=self.jobs[ident]['progress'])
        return result

    def _guard_football_arms(self, plan, *, lowered=False):
        from roki_ng.body.motion.gait import WALK_CLAVICLE_RAD, WALK_ELBOW_RAD
        angles = [0.] * 21
        if lowered:
            # Straight elbows with a small shoulder offset keep the landing balanced.
            angles[9], angles[20] = .30, -.30
        else:
            angles[6], angles[17] = WALK_ELBOW_RAD, -WALK_ELBOW_RAD
            angles[9], angles[20] = -WALK_CLAVICLE_RAD, WALK_CLAVICLE_RAD
        targets = {(v.Id, v.Sio): v.Data for v in
                   self.model.mapping.targets(self.model, angles, scale=1698)
                   if 1 <= v.Id <= 4}
        if any(self.sent_targets.get(key) != value for key, value in targets.items()):
            yield 'servo', [SimpleNamespace(Id=j, Sio=b, Data=v)
                            for (j, b), v in targets.items()], 15, 14
            yield 'drain',
        for command in plan:
            if command[0] == 'servo':
                _, values, frames, pause = command
                if not getattr(self, 'executing_gait_cycle', False):
                    values = [SimpleNamespace(Id=v.Id, Sio=v.Sio, Data=targets[v.Id, v.Sio])
                              if (v.Id, v.Sio) in targets else v for v in values]
                command = ('servo', values, frames, pause)
            yield command

    def _finish(self, status, reason=None):
        self.calibration_walking = False
        self.step_adaptation = None
        if self.active:
            job = self.jobs[self.active]
            if job["operation"] in ("test.start", "game.walk", "game.jump") and self.stop_requested and status == "completed":
                status, reason = "cancelled", "graceful_stop"
            if "segments" in job and self.segment_active:
                job["execution_uncertain"] = True
            if job.get('kick_type') == 'controller' and status != 'completed' and self.hardware.controller_pending:
                job['execution_uncertain'] = True
            if job.get('kick_type') == 'controller' and self.stop_requested and status == 'completed':
                status, reason = 'cancelled', 'graceful_stop'
            job.update(status=status, reason=reason, pose=self.pose)
            if status == "completed" and job.get("calibration") and not job.get('saved') and self.calibration_pending != self.active:
                job["status"] = "saving"
                self.calibration_pending = self.active
                self.emit("calibration.ready", {"job_id": self.active, "values": job["calibration"]})
            else:
                if not job.get('saved') and self.calibration_pending != self.active:
                    job.pop("calibration", None)
                self.emit("job.failed" if status == "failed" else "job.completed", dict(job))
        self.active = self.plan = self.instruction = None
        self.segment_active = False
        self.stop_requested = False

    def tick(self):
        if self.motherboard_reset_pending:
            self._publish_body_telemetry()
            return
        if self.imu:
            try:
                self.imu.tick()
            except Exception as exc:
                self.imu_error = str(exc)[:240]
                try:
                    self._command("imu.stop", {})
                except Exception as stop_error:
                    self.log("WARNING", f"IMU stop failed: {stop_error}")
                self.log("ERROR", f"IMU stopped: {self.imu_error}")
                self.emit("imu.fault", self.state())
        now = time.monotonic()
        if not self.body_disabled and now >= self.reconnect_at:
            self.reconnect_at = now + (1.0 if self.body_connected else self.retry_delay)
            try:
                if self.body_connected:
                    self.hardware.probe()
                else:
                    self.hardware.connect_body()
                    self.probe_successes += 1
                    if self.probe_successes < 3:
                        self.reconnect_at = time.monotonic() + 1.0
                        return
                    self.body_connected = True
                    self.error = None
                    self.important_errors.clear()
                    self.queue_reset_pending = False
                    self.retry_delay = 1.0
                    self.log("INFO", "Body connected; no previous motion resumed")
                    self.emit("body.connection", self.state())
            except (Fault, OSError) as exc:
                if isinstance(exc, Fault) and exc.code == "body_busy":
                    self.log("WARNING", f"Body probe deferred: {exc}")
                    self.reconnect_at = time.monotonic() + 0.05
                else:
                    self._important_failure(exc)
                    return
        if not self.body_connected:
            self._publish_body_telemetry()
            return
        if self.queue_reset_pending:
            if now >= self.queue_reset_at:
                try:
                    self._reset_after_motion_fault()
                    self.queue_reset_pending = False
                except (Fault, OSError) as exc:
                    self.queue_reset_at = time.monotonic() + 0.05
                    if isinstance(exc, Fault) and exc.code == 'body_busy':
                        self.log('WARNING', f'Body stop deferred until direct transaction resolves: {exc}')
                    else:
                        self._important_failure(exc, reset=False)
            return
        try:
            # Read queued execution failures even when the last job has ended.
            self.hardware.check_queue()
        except (Fault, OSError) as exc:
            self._important_failure(exc)
            self._publish_body_telemetry()
            return
        previous_sequence = self.body_imu.sequence
        self._tick_body_imu()
        if (self.step_adaptation is not None and self.body_imu.sequence != previous_sequence
                and self.body_imu.fresh(time.monotonic())):
            self.step_adaptation.observe(self.body_imu.pitch, self.body_imu.roll)
        self._tick_body_servos()
        self._tick_body_power()
        if not self.body_connected:
            return
        if not self.plan:
            self._tick_stabilization()
            self._publish_body_telemetry()
            return
        adaptive_walk = (self.active is not None and self.parameters.get("stabilization.stand_walk")
                         and self.parameters["stabilization.enabled"]
                         and (self.jobs[self.active]["operation"] in ("motion.drive", "game.walk")
                              or self.calibration_walking))
        if not adaptive_walk:
            self.stabilizer.freeze("motion")
        try:
            self._monitor_motion_posture()
            if time.monotonic() < self.next_at:
                return
            if self.instruction is None:
                self.instruction = next(self.plan)
                self.instruction_started = time.monotonic()
            kind, *args = self.instruction
            if kind == "imu":
                after = args[0]
                ready = (self.body_imu.fresh(time.monotonic())
                         and (after is None or self.body_imu.sequence > after))
                if not ready and not self.stop_requested:
                    if time.monotonic() - self.instruction_started > 1:
                        raise Fault("imu_stale", "No fresh polled body IMU within 1 second")
                    self.next_at = time.monotonic() + 0.005
                    return
            elif kind == "drain":
                if not self.hardware.drained():
                    if time.monotonic() - self.instruction_started > 3:
                        raise Fault("hardware_error", "Body queue did not drain within 3 seconds")
                    self.next_at = time.monotonic() + 0.005
                    return
            elif kind == "servo":
                values, frames, pause = args
                if not values or not 1 <= frames <= 255 or not 0 <= pause <= 255:
                    raise Fault("invalid_motion", "Invalid servo frame")
                if any(not 0 <= v.Data <= 16383 for v in values):
                    raise Fault("invalid_motion", "Servo position outside protocol range")
                correction = None
                if adaptive_walk and {(7, 1), (7, 2)}.issubset((v.Id, v.Sio) for v in values):
                    correction = self.stabilizer.propose(self.body_imu, self.parameters,
                                                         time.monotonic(), walking=True)
                sent_at = time.monotonic()
                self._send_targets(values, frames, pause, correction=correction)
                # Start-to-start period; no burst catch-up after late frames.
                self.next_at = sent_at + self.hardware.frame_s * (pause + 1)
            elif kind == 'controller_write':
                self.hardware.controller_write(*args)
            elif kind == 'controller_start':
                if not self.stop_requested:
                    self.hardware.controller_start()
            elif kind == 'controller_wait':
                witness = args[0]
                if time.monotonic() - self.instruction_started > 10:
                    raise Fault('hardware_error', 'Controller kick completion not confirmed within 10 seconds')
                try:
                    complete = witness.poll(self.hardware)
                except (Fault, OSError) as exc:
                    if isinstance(exc, Fault) and exc.code not in ('body_busy', 'hardware_error'):
                        raise
                    # Read-only telemetry is not an important motion failure.
                    self.log('WARNING', f'Controller kick status unavailable: {exc}')
                    witness.values = []
                    complete = False
                if not complete:
                    self.next_at = time.monotonic() + .02
                    return
                self.hardware.controller_completed()
            elif kind == "sleep":
                self.next_at = time.monotonic() + max(0, args[0])
            self.instruction = None
        except StopIteration:
            self._finish("completed")
        except Exception as exc:
            if isinstance(exc, Fault) and exc.code == "body_busy" and self.instruction is not None:
                self.log("WARNING", f"Body command deferred before execution: {exc}")
                self.next_at = time.monotonic() + 0.02
                return
            if isinstance(exc, OSError) or isinstance(exc, Fault) and exc.code == "hardware_error":
                self._important_failure(exc)
                return
            self.pose = "unknown"
            if self.active and self.jobs[self.active]['operation'] == 'motion.drive':
                self.drive = None
                self.drive_rearm_required = True
            self.engine = None
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate("motion_fault")
            self.stabilizer.reset("motion_fault")
            # Sensor/calibration/recovery failures fail the job, not the body
            # connection. In particular a failed get-up must remain retryable.
            retryable = isinstance(exc, Fault) and exc.code in (
                "imu_stale", "imu_discontinuous", "imu_invalid", "calibration_invalid",
                "course_not_reached", "posture_uncertain", "posture_unstable", "get_up_failed", "fallen")
            self.error = None if retryable else str(exc)
            self.log("ERROR", str(exc))
            self._finish("failed", str(exc))
            try:
                self._reset_after_motion_fault()
            except Exception as reset_error:
                if isinstance(reset_error, Fault) and reset_error.code == 'body_busy':
                    self.log('WARNING', f'Body stop deferred until direct transaction resolves: {reset_error}')
                    self.queue_reset_pending = True
                    self.queue_reset_at = time.monotonic() + .05
                else:
                    self._important_failure(reset_error, reset=False)
        finally:
            self._publish_body_telemetry()

    def _reset_after_motion_fault(self):
        self.hardware.reset()
        if time.monotonic() < self.protect_head_until:
            neutral = [SimpleNamespace(Id=0, Sio=1, Data=7500),
                       SimpleNamespace(Id=12, Sio=2, Data=7500)]
            self._send_targets(neutral, 5, 4)
            self.head = {"pan": 0, "tilt": 0}
            self.head_settled_at = time.monotonic() + 5*self.hardware.frame_s + .4
            self.log("WARNING", "Forward fall: neutral head target sent")
        self.protect_head_until = 0.0

    def read_body_quaternion(self, max_age=BodyImu.MAX_AGE_S):
        return self.body_imu.sample(max_age)

    def wait_for_imu(self, after_sequence=None):
        if self.stop_requested:
            return False
        yield "imu", self.body_imu.sequence if after_sequence is None else after_sequence
        return not self.stop_requested

    def _monitor_motion_posture(self):
        from roki_ng.body.recovery import body_position
        job = self.jobs[self.active]
        manual = job['operation'] == 'motion.drive'
        if (job.get('kick_type') == 'controller' and self.hardware.controller_pending
                and not self.body_imu.fresh(time.monotonic())):
            raise Fault('imu_stale', 'Body IMU lost during controller kick')
        if manual and job.get('posture_verified') and not self.body_imu.fresh(time.monotonic()):
            raise Fault('imu_stale', 'Body IMU lost during manual walking; release controls to restart')
        if (not (manual or job.get('kick_type') == 'controller' or job["operation"].startswith('game.') or
                 job["operation"] == "test.start" and job.get("posture_verified"))
                or self.recovery_active or self.body_imu.sequence == self.fall_sequence
                or not self.body_imu.fresh(time.monotonic())):
            return
        self.fall_sequence = self.body_imu.sequence
        pitch = self.body_imu.pitch
        stamp = self.body_imu.received_at
        previous, self.previous_pitch = self.previous_pitch, (stamp, pitch)
        if pitch is not None and abs(self.body_imu.roll) < 40:
            rising = (previous and previous[1] is not None and 0 < stamp-previous[0] <= .15
                      and pitch-previous[1] > 30*(stamp-previous[0]))
            if pitch >= 35 or (pitch >= 20 and rising):
                self.protect_head_until = time.monotonic() + .5
                raise Fault("fallen", f"Forward fall at {pitch:.1f} degrees")
        try:
            position = body_position(self.body_imu.sample())
        except Fault:
            self.fall_samples = 0
            return
        self.fall_samples = self.fall_samples + 1 if position != "upright" else 0
        if self.fall_samples >= 2:
            raise Fault("fallen", f"Body fell {position}; motion cancelled")

    def _body_telemetry(self):
        now = time.monotonic()
        imu_state = self.body_imu.state(now)
        head = getattr(self.imu, 'latest', None) if self.imu else None
        imu_state['head_reference'] = dict(head, commanded_head=dict(self.head)) if head else None
        return {"body.imu": imu_state,
                "body.power": self.body_power.state(now, self.parameters["power.voltage_scale"]),
                "body.servos": self.body_servos.state(now),
                "body.stabilization": self.stabilizer.state(self.parameters) | {
                    "diagnostics": self.stabilization_diagnostics.result,
                    "source_mono_ns": int(now * 1e9), "valid": self.body_connected}}

    def _publish_body_telemetry(self):
        now = time.monotonic()
        if self.imu_display_status is not None:
            self.imu_display_status.update(self.body_imu.state(now), now)
        self.stabilization_diagnostics.tick(self, now)
        warning = self.stabilizer.reason if self.stabilizer.reason in (
            "imu_stale", "tilt_outside_range", "joint_limit") else (
                "saturated" if self.stabilizer.saturated and self.stabilizer.reason == "regulating" else None)
        if (self.parameters["stabilization.enabled"] and warning
                and warning != self.stabilization_warning and now >= self.stabilization_log_at):
            self.log("WARNING", f"Crouch stabilization: {warning}; "
                     f"correction={self.stabilizer.offset_deg:.3f} deg")
            self.stabilization_log_at = now + 1
        self.stabilization_warning = warning
        # Supervisor/game consumes the same cache even with no GUI subscriber.
        if now >= self.telemetry_at:
            self.telemetry_at = now + 0.1
            self.emit("body.telemetry", self._body_telemetry())

    def walk_stride_scale(self):
        if not self.parameters.get('walk.adaptive_stride'):
            return 1.0
        if self.step_adaptation is None:
            if not self.body_imu.fresh(time.monotonic()):
                return 1.0
            self.step_adaptation = StepAdaptation(self.body_imu.pitch, self.body_imu.roll,
                                                  self.parameters['walk.adaptive_excursion_deg'])
        scale = self.step_adaptation.next_cycle()
        self.jobs[self.active]['stride_scale'] = scale
        return scale

    def _tick_body_imu(self):
        now = time.monotonic()
        motion_due = (self.plan is not None and now >= self.next_at
                      and (self.instruction is None or self.instruction[0] == 'servo'))
        if isinstance(self.body_imu, FallbackImu):
            ready = not motion_due
            ready = ready and now >= self.body_imu.primary.next_at
            if ready:
                ready = self.hardware.telemetry_ready()
            self.body_imu.tick(self.hardware, getattr(self.imu, 'latest', None),
                               self.head, self.head_settled_at, bool(self.active),
                               primary_ready=ready)
            return
        # Servo deadlines take precedence. No backlog of missed polling periods.
        if now < self.body_imu.next_at or motion_due:
            return
        try:
            self.body_imu.next_at = now + BodyImu.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_imu.poll(self.hardware)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body IMU polling failed: {exc}")

    def _send_targets(self, values, frames, pause, *, correction=None):
        offsets = self.stabilizer.ticks(self.stabilizer.offset_deg if correction is None else correction)
        corrected = [SimpleNamespace(Id=v.Id, Sio=v.Sio,
                     Data=v.Data + offsets.get((v.Id, v.Sio), 0)) for v in values]
        if any(not 0 <= v.Data <= 16383 for v in corrected):
            raise Fault("invalid_motion", "Corrected servo position outside protocol range")
        for value in corrected:
            key = value.Id, value.Sio
            if offsets.get(key, 0):
                low, high = self.hip_limits[key]
                if not low <= value.Data <= high:
                    raise Fault("invalid_motion", "Frozen stabilization correction exceeds hip limit")
        self.hardware.send(corrected, frames, pause)
        if correction is not None:
            self.stabilizer.commit(correction)
        # Relative trajectories start from nominal targets, never corrected ones.
        self.servo_targets.update(((v.Id, v.Sio), v.Data) for v in values)
        self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in corrected)
        self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in corrected)

    def _tick_body_servos(self):
        now = time.monotonic()
        if (not self.body_connected
                or not (self.servo_watch or self.parameters["stabilization.diagnostics_enabled"])
                or now < self.body_servos.next_at
                or (self.plan and now >= self.next_at)):
            return
        try:
            self.body_servos.next_at = now + BodyServos.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_servos.read(self)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body positions polling failed: {exc}")

    def _tick_body_power(self):
        now = time.monotonic()
        if (not self.body_connected or not self.power_watch
                or now < self.body_power.next_at
                or (self.plan and now >= self.next_at)):
            return
        try:
            self.body_power.next_at = now + BodyPower.PERIOD_S
            if self.hardware.telemetry_ready():
                self.body_power.read(self.hardware)
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Body power polling failed: {exc}")

    def _release_stabilization(self):
        # An absolute trajectory owns its next interpolated target. Do not send a
        # separate zero-correction pose before it. No physical motion happens here.
        self.servo_targets.update(self.sent_targets)
        self.stabilizer.reset("absolute_motion")

    def _tick_stabilization(self):
        regulator = self.stabilizer
        if not self.parameters["stabilization.enabled"]:
            regulator.freeze("disabled")
            return
        standing = self.parameters.get("stabilization.stand_walk") and self.pose in ("stand", "base_stand")
        if (self.pose not in ("crouch", "crouch_centered") and not standing) or self.error:
            regulator.freeze("pose_not_supported")
            return
        hips = ((7, 1), (7, 2))
        if any(key not in self.servo_targets for key in hips):
            regulator.freeze("missing_targets")
            return
        try:
            if not self.hardware.drained():
                regulator.freeze("queue_busy")
                return
            proposed = regulator.propose(self.body_imu, self.parameters, time.monotonic())
            if proposed is None:
                return
            offsets = regulator.ticks(proposed)
            # Respect the model's hip bounds as well as the wire range. The model
            # stores angles before FACTOR, so convert its limits to wire targets.
            targets = []
            for servo, bus in hips:
                data = self.servo_targets[servo, bus] + offsets[servo, bus]
                lo, hi = self.hip_limits[servo, bus]
                if not max(0, lo) <= data <= min(16383, hi):
                    regulator.freeze("joint_limit")
                    return
                targets.append(SimpleNamespace(Id=servo, Sio=bus, Data=data))
            if offsets == regulator.ticks(regulator.offset_deg):
                regulator.offset_deg = proposed  # Accumulate only sub-tick slew.
                return
            frames = max(1, math.ceil(BodyImu.PERIOD_S / self.hardware.frame_s))
            self.hardware.send(targets, frames, frames - 1)
            self.sent_targets.update(((v.Id, v.Sio), v.Data) for v in targets)
            self.target_times.update(((v.Id, v.Sio), time.monotonic()) for v in targets)
            regulator.commit(proposed)
        except (Fault, OSError) as exc:
            if isinstance(exc, Fault) and exc.code == "body_busy":
                self.log("WARNING", f"Stabilization deferred: {exc}")
                regulator.freeze("queue_busy")
            else:
                self._important_failure(exc)

    def _important_error_count(self):
        cutoff = time.monotonic() - 5
        while self.important_errors and self.important_errors[0] <= cutoff:
            self.important_errors.popleft()
        return len(self.important_errors)

    def _important_failure(self, exc, *, reset=True):
        count = min(3, getattr(exc, "count", 1))
        self._important_error_count()
        self.important_errors.extend([time.monotonic()] * count)
        self.last_important_error = str(exc)[:240]
        self.log("ERROR", f"Important body operation failed ({len(self.important_errors)}/3 in 5s): {exc}")
        if not self.body_connected or len(self.important_errors) >= 3:
            self._link_lost(exc)
            return
        # An uncertain command must never be retried. End this job and discard
        # its queued tail, but keep the worker/link usable below the threshold.
        self.pose, self.drive, self.engine = "unknown", None, None
        self.servo_targets.clear()
        self.sent_targets.clear()
        self.target_times.clear()
        self.body_servos.invalidate("uncertain_motion")
        self.stabilizer.reset("uncertain_motion")
        self._finish("failed", str(exc))
        if reset:
            try:
                self.hardware.reset()
                self.queue_reset_pending = False
            except (Fault, OSError) as reset_error:
                if isinstance(reset_error, Fault) and reset_error.code == 'body_busy':
                    self.log('WARNING', f'Body stop deferred until direct transaction resolves: {reset_error}')
                    self.queue_reset_pending = True
                    self.queue_reset_at = time.monotonic() + .05
                else:
                    self._important_failure(reset_error, reset=False)
        else:
            self.queue_reset_pending = True
            self.queue_reset_at = time.monotonic() + 0.05
        self.emit("body.connection", self.state())

    def _link_lost(self, exc):
        changed = self.body_connected or self.error is None
        self.body_connected = False
        self.probe_successes = 0
        self.pose = "unknown"
        self.drive = None
        self.engine = None
        self.servo_targets.clear()
        self.sent_targets.clear()
        self.target_times.clear()
        self.body_servos.invalidate("link_lost")
        self.body_imu.invalidate(str(exc)[:240])
        self.body_power.invalidate(str(exc)[:240])
        self.stabilizer.reset("link_lost")
        self.error = str(exc)[:240]
        self._finish("failed", self.error)
        transport_error = str(exc).lower()
        if not self.motherboard_reset_pending and (
                'acm' in transport_error or 'host rpc response timeout' in transport_error):
            self.motherboard_reset_pending = True
            # The supervisor stops camera/IMU, pulses GPIO, boots STM, then
            # asks this sole ACM owner to reopen. Never replay the old plan.
            self.emit('motherboard.reset_required', {'reason': self.error})
        try:
            self.hardware.reset()
        except (Fault, OSError) as reset_error:
            self.log("ERROR", f"Body queue reset during reconnect failed: {reset_error}")
        self.reconnect_at = time.monotonic() + self.retry_delay
        self.retry_delay = min(5.0, self.retry_delay * 2)
        if changed:
            self.log("ERROR", f"Body link unavailable: {self.error}; retrying, motion cancelled")
            self.emit("body.connection", self.state())

    def _head(self, pan, tilt, frames=10):
        self.head_settled_at = float('inf')
        values = [SimpleNamespace(Id=0, Sio=1, Data=7500 + pan),
                  SimpleNamespace(Id=12, Sio=2, Data=7500 + tilt)]
        yield "servo", values, frames, frames - 1
        yield "drain",
        self.head = {"pan": pan, "tilt": tilt}
        self.head_settled_at = time.monotonic() + .4

    def _pose(self, name, *, right_first=True):
        if (name == 'crouch' and self.parameters.get('walk.roki1_pult_enabled', False)
                and not getattr(self.engine, 'pult_walk', False)):
            self.engine = None
            self.pose = 'unknown'
        if name == "head_field":
            yield from self._head(0, self.parameters["head.field_tilt"])
            return
        if self.pose == name or (name == "stand" and self.pose == "base_stand"):
            return
        if name == "stand" and self.pose == "unknown":
            yield from self._pose("base_stand")
            return
        if name == "stand":
            self._release_stabilization()
        if name == "crouch_centered" or (name == "crouch" and self.pose in (
                "crouch_centered", "splits_small", "splits_big")):
            yield from self._static_crouch(name == "crouch_centered")
            return
        if name == "stand" and self.pose in ("splits_small", "splits_big", "crouch_centered"):
            _, values = self._crouch_target(True, height=self.backend.STAND_HEIGHT)
            # Match walk_Final_Pose's neutral arms at a reachable leg extension.
            for value in values:
                if value.Id in (1, 2, 3, 4):
                    value.Data = 7500
            frames = self.parameters["walk.crouch_transition_frames"]
            self.pose, self.engine = "unknown", None
            yield "servo", values, frames, frames - 1
            yield "drain",
            self.pose = "stand"
            return
        if name == "crouch":
            # Entering gait from another pose must not reuse its foot geometry.
            self.engine = None
        if name == "base_stand":
            rows = json.loads((self.backend.ASSETS / "slots/Initial_Pose.json").read_text())["Initial_Pose"]
            if self.pose == 'crouch':
                frames = self.parameters['walk.crouch_transition_frames']
                rows = [[max(row[0], frames), *row[1:]] for row in rows]
            yield from self._slot(rows)
            # Re-enable the head at its last commanded target after a hard stop.
            yield from self._head(self.head["pan"], self.head["tilt"])
        elif self.simulated:
            if name == "crouch":
                _, values = self._crouch_target(False)
                yield "servo", values, 4, 3
                yield "drain",
            else:
                yield "sleep", 0.08
        elif name == "crouch":
            engine = self._engine()
            engine.first_Leg_Is_Right_Leg = right_first
            yield from engine.walk_Initial_Pose(start_mixing=False)
            yield "drain",
        elif name == "stand":
            if self.pose != "crouch":
                raise Fault("invalid_state", "Walk final pose requires crouch")
            yield from self._engine().walk_Final_Pose()
            yield "drain",
        self.pose = name
        if name in ("stand", "base_stand"):
            self.engine = None

    def _complete_segment(self, delta, yaw):
        job = self.jobs[self.active]
        job['segments'].append(dict(index=len(job['segments']) + 1,
                                    dx_m=delta[0], dy_m=delta[1], imu_yaw_rad=yaw))
        job['progress'] = len(job['segments'])
        self.segment_active = False
        self.emit('job.progress', copy.deepcopy(job))

    def _settle_post_kick_crouch(self):
        deadline = time.monotonic() + 2
        window = collections.deque()
        while time.monotonic() < deadline:
            if not (yield from self.wait_for_imu()):
                return False
            stamp = self.body_imu.received_at
            pitch, roll = self.body_imu.pitch, self.body_imu.roll
            if abs(pitch) > 8 or abs(roll) > 8 or (window and stamp-window[-1][0] > .15):
                window.clear()
            if abs(pitch) <= 8 and abs(roll) <= 8:
                window.append((stamp, pitch, roll))
                while (max(v[1] for v in window)-min(v[1] for v in window) > 3
                       or max(v[2] for v in window)-min(v[2] for v in window) > 3):
                    window.popleft()
                if stamp-window[0][0] >= .25:
                    self.log("INFO", "Post-kick crouch steady; first step released")
                    return True
        raise Fault("posture_unstable", "Post-kick crouch did not settle before the first step")

    def _game_walk(self, kind, options):
        import roki_ng.body.motion.legacy_game_motion as legacy
        from roki_ng.body.motion.odometry import walk_delta
        from roki_ng.body.recovery import body_position
        def ball():
            if 0 <= time.monotonic()-self.game_ball_stamp <= .5:
                return self.game_ball
            return None
        p, a = self.parameters, options
        recipes = {
            'omni': lambda: legacy.omni(p, a['distance_m']*1000, a['course'], a['heading'], soft_touch=a.get('soft_touch', False), trim_cycles=a.get('trim_cycles', 0)),
            'approach': lambda: legacy.approach(p, a['distance_m'], a['heading'], ball,
                                                 stop_x_m=a.get('stop_x_m'),
                                                 target_y_m=a.get('target_y_m'), live_approach=a.get('live_approach', False)),
            'verify': lambda: legacy.verify(p, a['heading'], ball),
            'around': lambda: legacy.around(p, a['distance_m'], a['course'], a['initial_heading'], a['kick_heading'], ball),
            'kickoff_arc': lambda: legacy.kickoff_arc(p, a['distance_m'], a['course'], a['invert']),
            'kickoff': lambda: legacy.kickoff(p, a['invert'], ball),
        }
        prepared, completed, last = False, 0, None
        restart_after_kick = self.post_kick_walk
        approach_heading = None
        for order in recipes[kind]():
            if not (yield from self.wait_for_imu()):
                break
            quaternion = self.read_body_quaternion()
            if body_position(quaternion) != 'upright':
                raise Fault('fallen', 'Fall during original walking manoeuvre')
            if not prepared:
                self.segment_active = True
                if a.get('keep_crouch') and self.pose == 'crouch' and not self.simulated:
                    # Take the weight on the support leg without extending the knees.
                    engine = self._engine()
                    self.log('INFO', f'Walk continuation: retained_tilt={getattr(engine, "_walk_tilt", engine.xr):.4f}')
                    values = engine.crouch_target(right_first=order.right, preserve_tilt=True)
                    frames = self.parameters['walk.crouch_transition_frames']
                    yield 'servo', values, frames, frames-1
                    yield 'drain',
                else:
                    yield from self._pose('crouch', right_first=order.right)
                self.segment_active = False
                prepared = True
                if self.post_kick_walk:
                    if not (yield from self._settle_post_kick_crouch()):
                        break
                    if kind == 'approach' and ball() is None:
                        break
                    self.post_kick_walk = False
                if not (yield from self.wait_for_imu()):
                    break
                quaternion = self.read_body_quaternion()
            if a.get('live_approach'):
                order = legacy.live_ball_cycle(p, ball(), a['stop_x_m'], a['target_y_m'],
                                              order.index, order.total, order.right, order.heading)
                if order is None:
                    break
            yaw = quaternion_yaw(quaternion)
            field_yaw = wrap(yaw+a['yaw_offset'])
            if kind == 'approach':
                # Heading is latched after crouching, as in manual heading_hold.
                # The supervisor handles bearing changes between straight segments.
                if approach_heading is None:
                    approach_heading = yaw
                rotation = legacy.clip(wrap(approach_heading-yaw) * p['walk.heading_kp'],
                                       p['walk.heading_max_correction_rad'])
            else:
                rotation = legacy.clip(wrap(order.heading-field_yaw*order.yaw_gain), .3)
            if kind == 'approach':
                self.log('INFO', f'Approach cycle={order.index} step_mm={order.step:.3f} '
                         f'yaw={yaw:.4f} target={approach_heading:.4f} turn={rotation:.4f}')
            frame_stamp = self.game_ball_stamp
            if a.get('live_approach'):
                self.jobs[self.active]['last_approach_frame'] = frame_stamp
            self.segment_active = True
            step, side = yield from self._execute_walk_cycle(
                order.step, order.side, rotation*(1 if order.right else -1),
                order.index, order.total, order.right, half=order.half,
                lift_scale=min(1., .5 + .25*completed) if restart_after_kick else 1.)
            yield 'drain',
            delta = walk_delta(p, step, side, first=completed == 0, right_first=order.right)
            if order.half:
                delta = tuple(value*.5 for value in delta)
            self._complete_segment(delta, yaw)
            completed += 1
            last = order
            if kind == 'approach' and 'stop_x_m' in a:
                deadline = time.monotonic() + .65
                while (ball() is None or (a.get('live_approach') and self.game_ball_stamp <= frame_stamp)) and time.monotonic() < deadline and not self.stop_requested:
                    yield 'sleep', .025
        if prepared:
            self.segment_active = True
            # Normal recipes carry the original terminal cycle number. If
            # interrupted mid-series, settle both feet before leaving the gait.
            if last and (last.half or last.index != last.total-1) and not self.simulated:
                yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
                yield 'drain',
            self.pose = 'crouch'
            if a.get('keep_crouch') and not self.stop_requested:
                self.segment_active = False
                return
            # Roki2's base slot interpolates directly from the settled feet.
            # The intermediate IK extension pitches the body back on stopping.
            if self.backend.NAME != 'roki2':
                yield from self._pose('stand')
            yield from self._pose('base_stand')
            self.segment_active = False

    def _game_jump(self, direction, fraction):
        from roki_ng.body.motion.odometry import jump_delta
        from roki_ng.body.recovery import body_position
        if not (yield from self.wait_for_imu()):
            return
        if body_position(self.read_body_quaternion()) != 'upright':
            raise Fault('fallen', 'Game jump requires upright body')
        yaw = quaternion_yaw(self.read_body_quaternion())
        self.segment_active = True
        rows = self._jump_rows(direction, fraction)
        if direction.startswith('turn_'):
            rows.append([10] + [0] * 21)  # Same settled finish as rotation calibration.
        yield from self._slot(rows, legs_only=direction.startswith('turn_'))
        self._complete_segment(jump_delta(self.parameters, direction, fraction), yaw)
        # Absolute calibrated rows finish in the neutral stance, not manual crouch.
        self.pose = 'stand'

    def _execute_walk_cycle(self, step, side, rotation, index, total, right_first, *, half=False, lift_scale=1.):
        scale = self.walk_stride_scale()
        step, side = step * scale, side * scale
        if self.simulated:
            yield 'sleep', .08
        else:
            engine = self._engine()
            engine.first_Leg_Is_Right_Leg = right_first
            self.executing_gait_cycle = True
            try:
                if lift_scale == 1.:
                    yield from engine.walk_Cycle(step, side, rotation, index, total, half=half)
                else:
                    normal_height = engine.stepHeight
                    try:
                        engine.stepHeight = normal_height * lift_scale
                        yield from engine.walk_Cycle(step, side, rotation, index, total, half=half)
                    finally:
                        engine.stepHeight = normal_height
            finally:
                self.executing_gait_cycle = False
        return step, side

    def _walk(self, cycles=None, fixed=None):
        # Manual mode never gets up implicitly. No preparation is sent until
        # three distinct polling samples agree that the body is upright.
        from roki_ng.body.recovery import stable_position
        position = yield from stable_position(self)
        if position is None:
            return
        if position != 'upright':
            raise Fault('fallen', f'Manual walking requires upright body, got {position}')
        self.jobs[self.active]['posture_verified'] = True
        if fixed is None and self.parameters.get('walk.roki1_pult_enabled', False) and not self.simulated:
            yield from self._walk_pult()
            return
        initial_drive = fixed or self.drive
        right_first = not initial_drive or initial_drive["y"] <= 0
        if self.pose != "crouch":
            yield from self._pose("crouch", right_first=right_first)
        cycle = 0
        crouch = (fixed or self.drive or {}).get("crouch", "off")
        heading = None
        while True:
            drive = fixed if fixed is not None else self.drive
            active = fixed is not None or (drive and time.monotonic() < self.drive_deadline
                       and any(abs(drive[a]) > 0.05 for a in ("x", "y", "yaw")))
            if self.stop_requested or not active or (cycles is not None and cycle >= cycles):
                break
            crouch = drive["crouch"]
            if abs(drive["y"]) > 0.05:
                right_first = drive["y"] < 0
            rotation = drive["yaw"] * self.parameters["walk.max_yaw_rad"] * drive["speed"]
            if drive.get("heading_hold", False) and abs(drive["yaw"]) <= 0.05:
                if not (yield from self.wait_for_imu()):
                    break
                measured = quaternion_yaw(self.read_body_quaternion())
                if heading is None:
                    heading = measured
                limit = self.parameters["walk.heading_max_correction_rad"]
                rotation = max(-limit, min(limit, wrap(measured - heading)
                    * self.parameters["walk.heading_kp"] * (-1 if right_first else 1)))
            else:
                heading = None
            executed_step, _ = yield from self._execute_walk_cycle(
                drive["x"] * self.parameters["walk.max_step_mm"] * drive["speed"],
                abs(drive["y"]) * self.parameters["walk.max_side_mm"] * drive["speed"],
                rotation, cycle, 1000000, right_first)
            if drive.get("heading_hold", False):
                yield "drain",
            cycle += 1
            if fixed is not None and "run_command_mm" in self.jobs[self.active]:
                self.jobs[self.active]["run_command_mm"] += executed_step
            self.jobs[self.active]["progress"] = cycle
            self.emit("job.progress", dict(self.jobs[self.active]))
        # The ordinary last gait frame may have a raised foot. Explicit terminal cycle.
        if cycle and not self.simulated:
            yield from self._engine().walk_Cycle(0, 0, 0, 0, 1)
        yield "drain",
        self.pose = "crouch"
        if crouch == "centered":
            yield from self._static_crouch(True)
        elif crouch == "off":
            target = "base_stand" if self.backend.NAME == "roki2" else "stand"
            yield from self._pose(target)

    def _walk_pult(self):
        from roki_ng.body.motion.backends.roki1.walk_pult import PultWalk, drive_order
        engine = PultWalk(self.parameters, self.log, backend=self.backend)
        self._release_stabilization()
        self.engine = None
        self.pose = 'unknown'
        self.log('INFO', 'Experimental Roki1 walk_pult: manual fixed-step walk')
        completed = False
        crouch = (self.drive or {}).get('crouch', 'off')
        try:
            _, _, right, _ = drive_order(self.drive)
            engine.first_Leg_Is_Right_Leg = True if right is None else right
            yield from engine.initial()
            if not (yield from self.wait_for_imu()):
                return
            heading = quaternion_yaw(self.read_body_quaternion()) - .02
            cycle = 0
            stop_reason = 'operator_stop'
            while not self.stop_requested:
                drive = self.drive
                if not drive:
                    stop_reason = 'controls_released'
                    break
                crouch = drive['crouch']
                if time.monotonic() >= self.drive_deadline:
                    stop_reason = 'drive_timeout_700ms'
                    break
                if not any(abs(drive[a]) >= .05 for a in ('x', 'y', 'yaw')):
                    stop_reason = 'zero_input'
                    break
                step, side, right, turn = drive_order(drive)
                heading = wrap(heading + turn)
                if not self.body_imu.fresh(time.monotonic()):
                    if not (yield from self.wait_for_imu()):
                        return
                rotation = max(-.7, min(.7, wrap(heading-quaternion_yaw(self.read_body_quaternion()))))
                if cycle < 2:
                    step *= (cycle+1)/3
                if right is not None and right != engine.first_Leg_Is_Right_Leg:
                    if cycle >= 2:
                        yield from engine.cycle(step, side, rotation, half=True)
                    engine.first_Leg_Is_Right_Leg = right
                yield from engine.cycle(step, side, rotation, 'first' if cycle == 0 else 'other')
                cycle += 1
                self.jobs[self.active]['progress'] = cycle
                self.emit('job.progress', dict(self.jobs[self.active]))
            self.log('INFO', f'Walk_pult finishing: {stop_reason}; cycles={cycle}; IK skipped={engine.exitFlag}')
            if cycle:
                yield from engine.cycle(*engine.last_order, kind='last')
            if crouch == 'off':
                yield from engine.final()
                self.pose = 'stand'
            else:
                if crouch == 'centered':
                    yield from engine.centered_crouch()
                yield 'drain',
                self.pose = 'crouch_centered' if crouch == 'centered' else 'crouch'
                self.engine = engine
            completed = True
        finally:
            if not completed:
                self.engine = None
                self.pose = 'unknown'

    def _crouch_target(self, centered, height=None):
        if self.simulated:
            # Protocol simulation has no IK or physical geometry. These neutral
            # targets exist only to exercise selective-joint command bookkeeping.
            addresses = {(s, b) for s, b, *_ in self.model.ACTIVESERVOS[:21]}
            if self.model.mapping.split_knee:
                addresses.update(((13, 1), (13, 2)))
            return None, [SimpleNamespace(Id=s, Sio=b, Data=7500)
                          for s, b in sorted(addresses)]
        from roki_ng.body.motion.engine import Engine
        engine = Engine(self.parameters, log=self.log, backend=self.backend)
        if height is not None:
            engine.gaitHeight = height
        return engine, engine.crouch_target(centered)

    def _static_crouch(self, centered, frames=None):
        engine, values = self._crouch_target(centered)
        frames = frames or self.parameters["walk.crouch_transition_frames"]
        self.pose, self.engine = "unknown", None
        yield "servo", values, frames, frames - 1
        yield "drain",
        self.pose = "crouch_centered" if centered else "crouch"
        self.engine = engine

    def _prepare_jump(self, direction, fraction, target):
        yield from self._static_crouch(target == "crouch_centered")
        yield from self._relative_jump(self._relative_jump_steps(direction, fraction))

    def _individual_slot(self, steps):
        self._release_stabilization()
        self.pose, self.engine = "unknown", None
        yield from steps
        yield "drain",

    def _splits(self, kind, crouch):
        steps = slots.decode(self.backend.splits(kind == "big"), self.model)
        # Always prepare before the deep entry, including a request from stand.
        target = "crouch_centered" if crouch == "centered" else "crouch"
        if self.pose != target:
            yield from self._static_crouch(target == "crouch_centered")
        yield from self._individual_slot(steps)
        self.pose = f"splits_{kind}"

    def _get_up(self, crouch):
        from roki_ng.body.recovery import stable_position
        position = yield from stable_position(self)
        if position is None:
            return
        job = self.jobs[self.active]
        job.update(initial_posture=position, stage="getting_up")
        self.emit("job.progress", dict(job))
        target = "crouch_centered" if crouch == "centered" else "crouch"
        if position == "upright":
            yield from self._pose("base_stand" if crouch == "off" else target)
            return
        name = self.backend.RECOVERY[position]
        rows = json.loads((self.backend.ASSETS / "slots" / f"{name}.json").read_text())[name]
        self.recovery_active = True
        try:
            if crouch == "off":
                yield from self._slot(rows)
                self.pose = "stand"
            else:
                engine, values = self._crouch_target(crouch == "centered")
                steps = list(self._slot(rows))
                # Side recovery has straight legs in every row: fold them from
                # its first frame, retaining the original arm support sequence.
                if position in ("left", "right"):
                    legs = [v for v in values if 5 <= v.Id <= 10 or v.Id == 13]
                    for index, step in enumerate(steps[:-1]):
                        _, original, frames, pause = step
                        steps[index] = ("servo", [v for v in original
                            if not (5 <= v.Id <= 10 or v.Id == 13)] + legs, frames, pause)
                # Back/stomach recovery ends directly in the calculated crouch,
                # not in the old straight-leg last frame followed by sitting down.
                frames = int(rows[-1][0])
                steps[-2] = ("servo", values, frames, frames - 1)
                yield from steps
                self.engine, self.pose = engine, target
            verified = yield from stable_position(self)
            if verified != "upright":
                raise Fault("get_up_failed", f"Body is still {verified}; no automatic retry")
            job.update(stage="upright", posture_verified=True)
            self.emit("job.progress", dict(job))
        finally:
            self.recovery_active = False

    def _kick(self, leg, power, offset, kick_type=None, *, soft=False):
        kick_type = kick_type or self.parameters['motion.kick_type']
        if kick_type == 'controller' and not self.backend.CONTROLLER_KICK:
            raise Fault('unsupported', 'Controller kick slot contract is not verified for this robot model')
        if self.active:
            self.jobs[self.active].update(kick_type=kick_type, execution_uncertain=False)
            self.emit('job.progress', dict(self.jobs[self.active]))
        self._release_stabilization()
        self.engine = None
        if kick_type == 'software' and self.pose == 'crouch' and getattr(self, 'football_arms', False):
            yield from self._pose('base_stand')
        if kick_type == 'controller':
            from roki_ng.body.recovery import body_position
            yield 'drain',
            yield 'imu', self.body_imu.sequence
            if self.stop_requested:
                return
            if body_position(self.body_imu.sample()) != 'upright':
                raise Fault('posture_uncertain', 'Controller kick requires fresh upright IMU')
            if self.pose != 'base_stand':
                yield from self._pose('base_stand')
                yield 'imu', self.body_imu.sequence
                if body_position(self.body_imu.sample()) != 'upright':
                    raise Fault('posture_uncertain', 'Controller kick preparation lost upright posture')
            self.pose = 'unknown'
            # Zubr owns these targets now; do not retain a pre-kick pose as feedback.
            self.servo_targets.clear()
            self.sent_targets.clear()
            self.target_times.clear()
            self.body_servos.invalidate('controller_kick')
            for address, key in KICK_PARAMETERS:
                if self.stop_requested:
                    return
                yield 'controller_write', address, {'right': int(leg == 'right'), 'power': power, 'offset': offset}[key]
            if self.stop_requested:
                return
            yield 'controller_start',
            if not self.hardware.controller_pending:
                return
            yield 'controller_wait', KickWitness()
            yield 'imu', self.body_imu.sequence
            if body_position(self.body_imu.sample()) != 'upright':
                raise Fault('posture_uncertain', 'Controller kick ended without upright posture')
        elif self.simulated:
            self.pose = 'unknown'
            yield "sleep", 0.15
        else:
            self.pose = 'unknown'
            from roki_ng.body.motion.engine import Engine
            engine = Engine(self.parameters, log=self.log, backend=self.backend)
            engine.kick_power = power
            yield from engine.kick(leg == "right", small=soft, kick_offset=offset)
            yield "drain",
        self.pose = "stand"
        self.post_kick_walk = True
        self.engine = None

    def _validate_rows(self, rows, factor=1):
        if not isinstance(rows, list) or not 1 <= len(rows) <= 2048:
            raise Fault("invalid_motion", "Invalid slot rows")
        for row in rows:
            if not isinstance(row, list) or not 2 <= len(row) <= len(self.model.ACTIVESERVOS) + 1:
                raise Fault("invalid_motion", "Invalid slot row")
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in row):
                raise Fault("invalid_motion", "Non-numeric slot value")
            if not 1 <= round(row[0] / factor) <= 255:
                raise Fault("invalid_motion", "Slot duration outside 1..255 frames")

    def _slot(self, rows, factor=1, *, legs_only=False):
        self._validate_rows(rows, factor)
        self._release_stabilization()
        self.pose = "unknown"
        self.engine = None
        for row in rows:
            values = self.model.mapping.targets(self.model, row[1:], legs_only=legs_only)
            frames = round(row[0] / factor)
            yield "servo", values, frames, frames - 1
        yield "drain",

    def _relative_jump_steps(self, direction, fraction):
        if self.pose not in ("crouch", "crouch_centered", "base_stand"):
            raise Fault("invalid_state", "Relative jump requires crouch or base_stand")
        ids = {5, 10} if direction.startswith("turn_") else (
            {9, 10} if direction in ("forward", "backward") else {6, 10})
        selected = [(i, servo, bus, sign) for i, (servo, bus, sign, *_) in
                    enumerate(self.model.ACTIVESERVOS) if servo in ids]
        if any((servo, bus) not in self.servo_targets for _, servo, bus, _ in selected):
            raise Fault("invalid_state", "Missing commanded joint positions; select crouch first")
        baseline = dict(self.servo_targets)
        rows = self._jump_rows(direction, fraction)
        if direction.startswith("turn_"):
            # No neutral-pose preparation before a relative turn.
            rows = rows[1:]
        elif direction in ("forward", "backward"):
            # Legacy first row puts the left ankle value in the knee column.
            # For the relative variant mirror the right ankle, never move knees.
            for row in rows:
                row[13] = -row[2]
        steps = []
        for row in rows:
            values = []
            for index, servo, bus, sign in selected:
                target = baseline[servo, bus] + round(row[index + 1] * sign)
                if not 0 <= target <= 16383:
                    raise Fault("invalid_motion", "Relative jump exceeds servo protocol range")
                values.append(SimpleNamespace(Id=servo, Sio=bus, Data=target))
            frames = round(row[0])
            steps.append(("servo", values, frames, frames - 1))
        return steps

    def _relative_jump(self, steps):
        engine = self.engine
        pose = self.pose
        self.pose = "unknown"
        self.engine = None
        yield from steps
        yield "drain",
        # Only a completed return to the baseline may reuse the gait state.
        self.pose = pose
        self.engine = engine

    def _pult_turn(self, direction, fraction, crouch):
        from roki_ng.body.recovery import body_position, stable_position
        from roki_ng.body.motion.backends.roki1.primitives import pult_turn_steps
        # A completed support pose and fresh polled IMU already establish the
        # precondition. Do not wait for three MORE samples for every Q/E press.
        if (self.pose in ('stand', 'base_stand', 'crouch', 'crouch_centered')
                and self.body_imu.fresh(time.monotonic())):
            position = body_position(self.read_body_quaternion())
        else:
            position = yield from stable_position(self)
        if position is None:
            return
        if position != 'upright':
            raise Fault('fallen', 'Manual turn requires upright body')
        if crouch == 'off':
            yield from self._pose('stand')
        else:
            target = 'crouch_centered' if crouch == 'centered' else 'crouch'
            if self.pose != target:
                yield from self._pose('crouch')
                if crouch == 'centered' and not self.simulated:
                    yield from self._engine().centered_crouch()
                self.pose = target
        steps = pult_turn_steps(self.model, self.servo_targets, direction, fraction)
        self.log('INFO', f'Walk_pult manual turn: {direction}, fraction={fraction:.2f}')
        yield from self._relative_jump(steps)
        yield 'sleep', .06

    def _jump_rows(self, direction, fraction):
        return self.backend.jump_rows(direction, fraction)

    def close(self):
        try:
            self._command("imu.stop", {})
        except Exception as exc:
            self.log("WARNING", f"Could not stop capture: {exc}")
        try:
            self.command("motion.stop_hard", {})
        except (Fault, OSError) as exc:
            self.log("WARNING", f"Could not confirm body queue reset during shutdown: {exc}")
        if not self.simulated and hasattr(self.hardware, "mb"):
            self.hardware.mb.Close()
