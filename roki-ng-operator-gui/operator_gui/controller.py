"""Operator models; connection never acquires control or starts hardware."""

from collections import deque
from datetime import datetime
import json
import math
import time

from PySide6.QtCore import QObject, Property, QTimer, Signal, Slot

from .models import Rows, LogFilter, ParameterFilter
from .transport import Session
from .control import Control, scalar
from .data_sources import DataSources
from .video import Streams
from .camera import Camera
from .video_views import VideoViews
from .field_editor import FieldEditor
from .vision_tuning import VisionTuning
from .localisation import Localisation
from .game import Game


def pretty(value):
    return json.dumps(value, ensure_ascii=False, indent=2)


class Controller(QObject):
    changed = Signal()
    calibrationChanged = Signal()
    tiltKeys = ("walk.body_tilt_forward", "kick.body_tilt", "kick.recovery_tilt")
    testMeasurementsChanged = Signal()
    parameterChanged = Signal()
    parameterGroupsChanged = Signal()

    def __init__(self, host, port, config_dir, parent=None):
        super().__init__(parent)
        self.config_dir = config_dir
        self.host, self.port = host, port
        self.transport = Session(self)
        self._transport_phase = self.transport.phase
        self.transport.changed.connect(self._transport_changed)
        self.transport.welcomed.connect(self._welcome)
        self.transport.response.connect(self._response)
        self.transport.failed.connect(self._failed)
        self.transport.notification.connect(self._notification)
        self.transport.diagnostic.connect(self._log)
        self.control = Control(self.transport, self._log, self)
        self.data_sources = DataSources(self.transport, self)
        self.game = Game(self.transport, self.control, self.data_sources, self)
        self.localisation = Localisation(self.transport, self.control, self.data_sources, self)
        self.control.barrierIssued.connect(self.localisation.barrier)
        self.field_editor = FieldEditor(self.transport,self.control,self)
        self.control.barrierIssued.connect(self._barrier)
        self.control.barrierIssued.connect(self.data_sources.barrier)
        self.logs = Rows(self)
        self.log_filter = LogFilter(self.logs, self)
        self.slots = Rows(self)
        self.tests = Rows(self)
        self.parameters = Rows(self)
        self.parameter_filter = ParameterFilter(self.parameters, self)
        self.workers = Rows(self)
        self.history = deque(maxlen=2000)
        self.logs_paused = False
        self.pages = {}
        self.kick_preview = {}
        self.calibration_state = {}
        self.calibration_readback = {}
        self.calibration_tilts = {}
        self.calibration_poll_pending = False
        self.robot = self.mode = ""
        self.owner = ""
        self.status_text = "Статус ещё не запрошен."
        self.test_text = "Запросите каталог и выберите тест, чтобы прочитать его описание."
        self.param_key = ""
        self.param_text = "Выберите ключ после запроса каталога."
        self.param_parts = {}
        self.test_schemas = {}
        self.test_measurements = {}
        self.test_measurement_saved = {}
        self.capabilities = {}
        self.capabilities_text = "Возможности ещё не запрошены."
        self.notice = ""
        self.last_status_at = None
        self.gaps = 0
        self.last_log_record = 0
        self.log_notify = QTimer(self)
        self.log_notify.setSingleShot(True)
        self.log_notify.setInterval(33)
        self.log_notify.timeout.connect(self.changed.emit)
        self.clock = QTimer(self)
        self.clock.setInterval(1000)
        self.clock.timeout.connect(self.changed.emit)
        self.clock.start()
        self._log("INFO", "Клиент запущен. Подключение только по явному действию оператора.")
        self.streams = Streams(self.transport, self.control, self._log, self)
        self.camera = Camera(self.transport, self.control, self)
        self.video_views = VideoViews(config_dir / "video-views.json", self)
        self.streams.openView.connect(self.video_views.show)
        self.vision_tuning = VisionTuning(self.transport, self.control, self.streams, self)

    @Property("QVariantMap", notify=changed)
    def view(self):
        phases = {"disconnected": "Не подключён", "connecting": "Подключение...",
                  "connected": "Подключён", "lost": "Связь потеряна"}
        t = self.transport
        return dict(host=self.host, port=self.port, phase=t.phase,
                    connection=phases[t.phase], connected=t.connected,
                    robot=self.robot or "Нет данных", mode=self.mode or "Нет данных",
                    owner=self.owner or "Не запрошено", rtt="-" if t.rtt_ms is None else str(t.rtt_ms),
                    statusText=self.status_text, testText=self.test_text,
                    paramText=self.param_text, paramKey=self.param_key,
                    parameter=self.param_parts, capabilities=self.capabilities,
                    capabilitiesText=self.capabilities_text, notice=self.notice,
                    catalogsBusy=bool(self.pages), logPaused=self.logs_paused,
                    logCount=len(self.history), gaps=self.gaps,
                    statusAge="-" if self.last_status_at is None else str(int(time.monotonic() - self.last_status_at)),
                    invalid=t.invalid_packets)

    @Property(str, constant=True)
    def layoutPath(self):
        return str(self.config_dir / "layout.json")

    @Property("QVariantMap", notify=parameterChanged)
    def parameter(self):
        return self.param_parts

    @Property("QStringList", notify=parameterGroupsChanged)
    def parameterGroups(self):
        return ["Все"] + sorted({item["name"].split(".", 1)[0] for item in self.parameters.items})

    @Slot(str, str)
    def filterParameters(self, group, search):
        group = "" if group == "Все" else group
        self.parameter_filter.configure(group, search)
        if self.param_key and ((group and self.param_key.split(".", 1)[0] != group)
                               or search.casefold() not in self.param_key.casefold()):
            self.param_key = ""
            self.param_parts = {}
            self.param_text = ""
            self.parameterChanged.emit()
            self.changed.emit()

    @Slot(str, int)
    def connectRobot(self, host, port):
        self.host, self.port = host.strip(), port
        self.notice = ""
        self.transport.connect_to(host, port)

    @Slot()
    def disconnectRobot(self):
        self.transport.close()
        self._log("INFO", "Сессия закрыта. Команды остановки игры/смены режима не отправлялись.")

    @Slot()
    def _transport_changed(self):
        # RTT and counters refresh on the UI clock, not on every RPC response.
        if self.transport.phase == self._transport_phase:
            return
        self._transport_phase = self.transport.phase
        if not self.transport.connected:
            self.pages.clear()
            self.robot = ""
            self.kick_preview = {}
            self.calibration_state = {}
            self.calibration_readback = {}
            self.calibration_tilts = {}
            self.calibration_poll_pending = False
            self.calibrationChanged.emit()
            if hasattr(self, "video_views"):
                self.video_views.clearSelections()
        self.changed.emit()

    @Slot(object)
    def _welcome(self, body):
        self.robot = str(body.get("robot_id", "unknown"))
        self.mode = str(body.get("state", "unknown"))
        self.owner = "Не запрошено"
        self.status_text = "Сессия установлена. Подробный статус запрашивается отдельно."
        self.capabilities_text = "Возможности ещё не запрошены."
        self.last_status_at = None
        self.last_log_record = self.gaps = 0
        self.param_parts = {}
        self.test_schemas.clear()
        self.test_measurements.clear()
        self.testMeasurementsChanged.emit()
        self.capabilities = {}
        self.control.mode = self.mode
        self.parameterChanged.emit()
        for model in (self.slots, self.tests, self.parameters, self.workers):
            model.replace([])
        self.parameter_filter.configure("", "")
        self.parameterGroupsChanged.emit()
        self.param_key = ""
        self.param_text = "Выберите ключ после запроса каталога."
        self.test_text = "Каталог тестов ещё не запрошен."
        self._log("INFO", f"Подключён {self.robot}. Сессия наблюдения, управление не захвачено.")
        self.transport.request("log.subscribe", {"level": "DEBUG", "sources": []})
        self.changed.emit()

    @Property("QVariantMap", notify=calibrationChanged)
    def calibration(self):
        return self.calibration_state

    @Property("QVariantMap", notify=calibrationChanged)
    def calibrationReadback(self):
        return self.calibration_readback

    @Property("QVariantMap", notify=calibrationChanged)
    def calibrationTilts(self):
        return self.calibration_tilts

    @property
    def simulator_connected(self):
        return self.robot == "SIM-ROKI2" and self.transport.connected

    @Slot()
    def refreshCalibration(self):
        if self.simulator_connected and not self.calibration_poll_pending:
            self.calibration_poll_pending = True
            self.transport.request("sim.calibration.status")

    @Slot(bool)
    def setCalibrationMode(self, enabled):
        if self.simulator_connected:
            self.control.command("sim.calibration.mode", {"enabled": enabled}, manual=False, job=False)

    @Slot(str, "QVariantMap")
    def startCalibrationTest(self, name, options):
        if self.simulator_connected and self.calibration_state.get("enabled"):
            self.control.command("test.start", dict(options, name=name))

    @Property("QVariantMap", notify=calibrationChanged)
    def kickPreview(self):
        return self.kick_preview

    @Slot("QVariantMap")
    def previewKick(self, options):
        if self.simulator_connected and self.calibration_state.get("enabled"):
            self.control.command("sim.kick.preview", options, job=False)

    @Slot(int)
    def saveStrikeLift(self, mm):
        if (self.simulator_connected and self.calibration_state.get("enabled")
                and not self.calibration_state.get("busy") and 32 <= mm <= 70):
            self.control.command("params.set", {"key": "kick.strike_lift_mm", "value": mm},
                                 job=False, context="cal-tilt:kick.strike_lift_mm")

    @Slot()
    def repeatCalibration(self):
        if self.simulator_connected and self.calibration_state.get("enabled"):
            self.control.command("sim.calibration.repeat", {})

    @Slot(int, "QVariantMap")
    def saveCalibration(self, job_id, values):
        result = self.calibration_state.get("result") or {}
        if not (self.simulator_connected and self.calibration_state.get("enabled")
                and result.get("job_id") == job_id and values
                and values.keys() == result.get("values", {}).keys()):
            return
        try:
            numbers = {}
            for key, value in values.items():
                number = float(str(value).replace(",", "."))
                if not math.isfinite(number) or (key.endswith("_mm") and not number.is_integer()):
                    raise ValueError(key)
                numbers[key] = int(number) if key.endswith("_mm") else number
        except (TypeError, ValueError, OverflowError):
            self.control.reject("Введите конечные числа, расстояния — в целых миллиметрах.")
            return
        self.control.command("sim.calibration.save", {"job_id": job_id, "values": numbers},
                             context=str(job_id))

    @Slot(str, str)
    def saveCalibrationTilt(self, key, text):
        if not (self.simulator_connected and self.calibration_state.get("enabled")
                and key in self.tiltKeys and key in self.calibration_tilts):
            return
        try:
            value = float(text.replace(",", "."))
            if not -0.3 <= value <= 0.3:
                raise ValueError(key)
        except ValueError:
            self.control.reject("Введите коэффициент от −0,3 до +0,3.")
            return
        self.control.command("params.set", {"key": key, "value": value}, context="cal-tilt:" + key)

    @Slot()
    def requestStatus(self):
        self.transport.request("system.status")

    @Slot()
    def requestCapabilities(self):
        self.transport.request("system.capabilities")

    @Slot(str)
    def requestCatalog(self, category):
        operations = {"slots": "motion.slots", "tests": "test.list", "parameters": "params.keys"}
        if category not in operations or category in self.pages or not self.transport.connected:
            return
        self.pages[category] = []
        self.transport.request(operations[category], {} if category == "tests" else {"offset": 0, "limit": 8},
                               f"catalog:{category}:0")
        self.changed.emit()

    @Slot(str)
    def describeTest(self, name):
        self.transport.request("test.describe", {"name": name}, name)

    @Slot(str)
    def inspectParameter(self, key):
        self.param_key = key
        self.param_parts = {}
        self.param_text = "Запрашивается..."
        self.parameterChanged.emit()
        self.changed.emit()
        self.transport.request("params.describe", {"key": key}, key)
        self.transport.request("params.get", {"key": key}, key)

    @Slot(str, "QVariant")
    def saveParameter(self, key, value):
        if key != self.param_key or "type" not in self.param_parts or "value" not in self.param_parts:
            return
        try:
            value = scalar(self.param_parts, value)
        except (ValueError, TypeError, OverflowError) as exc:
            self.control.reject(str(exc))
            return
        self.control.command("params.set", {"key": key, "value": value}, manual=False, job=False, context=key)

    @Property("QVariantMap", notify=testMeasurementsChanged)
    def testMeasurements(self):
        return self.test_measurements

    @Property("QVariantMap", notify=testMeasurementsChanged)
    def testMeasurementSaved(self):
        return self.test_measurement_saved

    @Slot(str, "QVariantList")
    def loadTestMeasurements(self, job_id, keys):
        if job_id in self.test_measurements:
            return
        self.test_measurements[job_id] = {}
        for key in keys:
            self.transport.request("params.describe", {"key": key}, f"test-input:{job_id}:{key}")

    @Slot(str, "QVariantMap")
    def measureTest(self, job_id, values):
        fields = self.test_measurements.get(job_id, {})
        if not fields or set(values) != set(fields):
            self.control.reject("Заполните все измерения этого теста.")
            return
        try:
            numbers = {key: scalar(fields[key], value) for key, value in values.items()}
        except (ValueError, TypeError, OverflowError) as exc:
            self.control.reject(str(exc))
            return
        self.control.command("test.measure", {"job_id": int(job_id), "values": numbers},
                             job=False, context="test-save:" + job_id)

    @Slot(str, "QVariantMap")
    def startTest(self, name, values):
        schema = self.test_schemas.get(name)
        if not schema:
            return
        try:
            args = {}
            for key, meta in schema.get("parameters", {}).items():
                if meta.get("when") and values.get("mode") != meta["when"]:
                    continue
                args[key] = scalar(meta, values.get(key, meta.get("default")))
            mode = args.get("mode")
            if mode in schema.get("unavailable_modes", {}):
                raise ValueError(schema["unavailable_modes"][mode])
        except (ValueError, TypeError, OverflowError) as exc:
            self.control.reject(str(exc))
            return
        self.control.command("test.start", dict(args, name=name))

    @Slot()
    def _barrier(self):
        self.calibration_poll_pending = False
        self.pages.clear()
        self.changed.emit()

    @Slot(str, object, str)
    def _response(self, op, result, context):
        try:
            if context.startswith("test-input:"):
                _, job_id, key = context.split(":", 2)
                self.test_measurements.setdefault(job_id, {})[key] = result
                self.testMeasurementsChanged.emit()
                return
            if context.startswith("test-readback:"):
                _, job_id = context.split(":", 1)
                self.test_measurement_saved[job_id] = result.get("values", {})
                self.testMeasurementsChanged.emit()
                return
            if context.startswith("test-save:"):
                job_id = context.split(":", 1)[1]
                self.transport.request("params.get", {"keys": list(result["saved"])}, "test-readback:" + job_id)
                self.notice = "Измерения сохранены в профиле робота."
                self._log("INFO", self.notice + " " + pretty(result))
                self.changed.emit()
                return
            if op == "sim.kick.preview":
                self.kick_preview = result
                self.calibrationChanged.emit()
                return
            if op.startswith("sim.calibration."):
                if op == "sim.calibration.status":
                    self.calibration_poll_pending = False
                if not self.simulator_connected:
                    return
                enabled = result.get("enabled", False)
                if enabled and not self.calibration_state.get("enabled"):
                    for key in (*self.tiltKeys, "kick.strike_lift_mm"):
                        self.transport.request("params.get", {"key": key}, "cal-tilt:" + key)
                trial = result.get("result") or {}
                old_trial = self.calibration_state.get("result") or {}
                if trial.get("kick_lab") and trial.get("job_id") != old_trial.get("job_id"):
                    self.kick_preview = trial["kick_lab"]["camera"]
                if trial.get("job_id") != old_trial.get("job_id") or not enabled:
                    self.calibration_readback = {}
                self.calibration_state = result
                if op == "sim.calibration.mode":
                    self.kick_preview = {}
                    self.requestStatus()
                if op == "sim.calibration.save":
                    for key in trial.get("values", {}):
                        self.transport.request("params.get", {"key": key}, f"cal-saved:{context}:{key}")
                self.calibrationChanged.emit()
                return
            elif op == "params.get" and context.startswith("cal-saved:"):
                _, job_id, key = context.split(":", 2)
                if str((self.calibration_state.get("result") or {}).get("job_id")) == job_id:
                    self.calibration_readback[key] = result["value"]
                    self.calibrationChanged.emit()
                return
            elif context.startswith("cal-tilt:") and self.simulator_connected:
                key = context.split(":", 1)[1]
                if op == "params.get":
                    self.calibration_tilts[key] = result["value"]
                    self.calibrationChanged.emit()
                elif op == "params.set":
                    self.transport.request("params.get", {"key": key}, context)
                return
            elif op in ("session.heartbeat", "mode.set"):
                mode = str(result.get("state", self.mode))
                if mode == self.mode:
                    return
                self.mode = mode
            elif op in ("game.start", "game.status") and result.get("running") is True:
                self.mode = "GAME"
            elif op == "system.status":
                self.mode = str(result.get("state", "unknown"))
                self.owner = "Нет владельца" if result.get("owner") is None else str(result["owner"])
                self.status_text = pretty(result)
                self.last_status_at = time.monotonic()
                self.workers.replace([dict(name=str(k), state=str(v.get("state", "unknown")), alive=bool(v.get("alive")))
                                      for k, v in result.get("workers", {}).items()])
            elif op == "system.capabilities":
                self.capabilities = result
                self.capabilities_text = pretty(result)
            elif context.startswith("catalog:"):
                _, category, offset = context.split(":")
                if category not in self.pages:
                    return
                items = result["items"]
                if not isinstance(items, list) or not all(isinstance(v, str) for v in items):
                    raise ValueError("Invalid catalog items")
                self.pages[category].extend(items)
                following = result.get("next_offset")
                if following is not None:
                    if type(following) is not int or following <= int(offset) or len(self.pages[category]) > 2000:
                        raise ValueError("Invalid catalog pagination")
                    self.transport.request(op, {"offset": following, "limit": 8}, f"catalog:{category}:{following}")
                else:
                    items = self.pages.pop(category)
                    getattr(self, category).replace([{"name": v} for v in items])
                    if category == "parameters":
                        self.parameterGroupsChanged.emit()
                    self._log("INFO", f"Получен каталог {category}: {len(items)}")
                    if category == "tests":
                        for name in items:
                            self.describeTest(name)
            elif op == "test.describe":
                self.test_text = pretty(result)
                self.test_schemas[context] = result
                self.tests.update_named(context, {"schema": result, "raw": pretty(result)})
            elif op in ("params.get", "params.describe", "params.set") and context == self.param_key:
                self.param_parts.update(result)
                self.param_text = pretty(self.param_parts)
                self.parameterChanged.emit()
                if op == "params.set":
                    self._log("INFO", f"Сохранён {context}: {result.get('value')}; применение: {result.get('apply')}")
            elif op == "log.subscribe":
                self._log("INFO", "Журнал робота подключён. Фильтры в панели работают локально.")
            else:
                return
        except (ValueError, TypeError, KeyError, AttributeError) as exc:
            self._failed(op, f"Invalid response: {exc}", context)
        self.changed.emit()

    @Slot(str, str, str)
    def _failed(self, op, message, context):
        if op == "sim.calibration.status":
            self.calibration_poll_pending = False
        if context.startswith("catalog:"):
            self.pages.pop(context.split(":")[1], None)
        self.notice = f"{op}: {message}"
        if op.startswith("params.") and context == self.param_key:
            self.param_text = self.notice
        self._log("ERROR", self.notice)
        self.changed.emit()

    @Slot(object)
    def _notification(self, batch):
        try:
            if batch["generation"] != self.transport.generation or not self.transport.connected:
                return
            if batch["dropped"]:
                self._log("WARNING", f"Входная очередь клиента переполнена: отброшено {batch['dropped']} сообщений")
            for message in batch["messages"]:
                self._handle_notification(message)
        finally:
            self.transport.deliveryDone.emit()

    def _handle_notification(self, message):
        op, body = message["op"], message["body"]
        if op == "data.sample":
            self.data_sources.notification(body)
            return
        self.control.notification(op, body)
        if op == "log.sample":
            dropped = body.get("dropped", 0)
            if type(dropped) is int and dropped > 0:
                self._log("WARNING", f"Сервер вытеснил {dropped} записей журнала")
            records = body.get("records", [])
            if not isinstance(records, list):
                return
            for record in records:
                if not isinstance(record, dict):
                    continue
                ident = record.get("record_sequence", 0)
                if type(ident) is not int or ident <= self.last_log_record:
                    continue
                self.last_log_record = ident
                self.game.record("robot.log", record)
                self._log(str(record.get("level", "INFO")), str(record.get("message", "")),
                          str(record.get("source", "robot")))
        else:
            self._log("INFO", f"{op}: {pretty(body)}", "robot/event")
    @Slot(str, str)
    def _log(self, level, message, source="operator"):
        record = dict(time=datetime.now().strftime("%H:%M:%S"), level=level,
                      source=source, message=message)
        self.history.append(record)
        if not self.logs_paused:
            self.logs.append(record)
        if not self.log_notify.isActive():
            self.log_notify.start()

    @Slot(bool)
    def pauseLogs(self, paused):
        self.logs_paused = paused
        if not paused:
            self.logs.replace(self.history)
        self.changed.emit()

    @Slot()
    def clearLogs(self):
        self.history.clear()
        self.logs.replace([])
        self.changed.emit()

    @Slot(str)
    def layoutResult(self, message):
        self._log("INFO", message)

    def shutdown(self):
        self.game.shutdown()
        self.vision_tuning.shutdown()
        self.localisation.shutdown()
        self.streams.shutdown()
        self.camera.shutdown()
        self.data_sources.shutdown()
        self.control.shutdown()
        self.clock.stop()
        self.log_notify.stop()
        self.transport.shutdown()
