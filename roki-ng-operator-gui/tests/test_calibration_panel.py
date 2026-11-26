"""Exercise the simulator-only Tests panel with a Qt client and a UDP fixture."""
import os
from pathlib import Path
import subprocess
import sys


def test_calibration_panel():
    run = subprocess.run([sys.executable, "-m", "tests.test_calibration_panel"],
                         env=dict(os.environ, QT_QPA_PLATFORM="offscreen"),
                         capture_output=True, text=True, timeout=30)
    assert run.returncode == 0, run.stdout + run.stderr


def exercise():
    import tempfile
    from PySide6.QtCore import QMetaObject, QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlComponent, QQmlEngine
    from PySide6.QtQuick import QQuickItem
    from operator_gui.controller import Controller
    from tests.fake_robot import FakeRobot
    from tests.test_operator import wait_until

    key = "motion.run_10_mm"

    class Simulator(FakeRobot):
        def __init__(self):
            self.state = dict(enabled=False, busy=False, phase="stand", result=None)
            super().__init__(robot_id="SIM-ROKI2")

        def _result(self, op, body):
            if op == "sim.calibration.mode":
                self.mode = "MANUAL"
                self.game_running = False
                self.state["enabled"] = body["enabled"]
            elif op == "sim.calibration.save":
                assert body == dict(job_id=7, values={key: 890})
                self.values.update(body["values"])
                self.state["result"].update(saved=True, values=body["values"])
            elif op == "test.start":
                assert body == dict(name="get_up_test", fixture="upright")
                self.state["phase"] = "rotation"
                self.jobs[1] = dict(job_id=1, status="completed", operation=op)
                return dict(accepted=True, job_id=1)
            elif op != "sim.calibration.status":
                return super()._result(op, body)
            return self.state

    def child(item, name):
        if item.objectName() == name:
            return item
        for sub in item.childItems():
            found = child(sub, name)
            if found is not None:
                return found

    def click(name):
        button = child(panel, name)
        assert button is not None and button.isEnabled(), name
        assert QMetaObject.invokeMethod(button, "clicked")

    app = QGuiApplication([])
    robot = Simulator()
    temp = tempfile.TemporaryDirectory()
    c = Controller("127.0.0.1", robot.port, Path(temp.name))
    engine = QQmlEngine()
    for name, obj in [("backend", c), ("controls", c.control), ("testsModel", c.tests), ("slotsModel", c.slots)]:
        engine.rootContext().setContextProperty(name, obj)
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(
        Path(__file__).resolve().parents[1] / "qml/CatalogPanel.qml")))
    panel = component.createWithInitialProperties(dict(kind="tests"))
    try:
        assert isinstance(panel, QQuickItem), component.errors()
        panel.setWidth(640)
        panel.setHeight(470)
        c.connectRobot("127.0.0.1", robot.port)
        wait_until(lambda: c.calibration.get("enabled") is False)
        assert not child(panel, "calibrationModeButton").isEnabled()
        c.control.acquire()
        wait_until(lambda: c.control.owns)
        click("calibrationModeButton")
        wait_until(lambda: child(panel, "calibrationStandButton") is not None and c.control.view["ready"])
        assert not child(panel, "calibrationShortButton").isEnabled()
        assert not c.game.view["canStart"]
        click("calibrationStandButton")
        wait_until(lambda: child(panel, "calibrationRotationButton").isEnabled())
        robot.state.update(phase="ready", result=dict(job_id=7, test="run_test", dx_mm=903,
                           dy_mm=0, yaw_deg=0, path_mm=904, upright=True, pitch_deg=2.1,
                           pitch_min_deg=-4.5, pitch_max_deg=8.3, values={key: 903},
                           previous={key: 800}))
        c.refreshCalibration()
        wait_until(lambda: child(panel, "calibrationValue_" + key) is not None)
        assert child(panel, "calibrationStandButton").isEnabled()
        assert child(panel, "calibrationSaveTilt_walk.body_tilt_forward").isEnabled()
        value = child(panel, "calibrationValue_" + key)
        assert value.property("text") == "903"
        pitch = child(panel, "calibrationPitch").property("text")
        assert all(angle in pitch for angle in ("+2.1°", "-4.5°", "+8.3°"))
        value.setProperty("text", "890.5")
        click("calibrationSaveButton")
        assert "целых миллиметрах" in c.control.error
        value.setProperty("text", "890")
        c.refreshCalibration()
        app.processEvents()
        assert value.property("text") == "890"
        click("calibrationSaveButton")
        wait_until(lambda: c.calibrationReadback.get(key) == 890)
        assert not child(panel, "calibrationSaveButton").isEnabled()
        click("footballModeButton")
        wait_until(lambda: not c.calibration["enabled"])
        wait_until(lambda: c.game.view["canStart"])
        assert not robot.game_running
        assert not robot.errors
    finally:
        c.shutdown()
        robot.close()
        temp.cleanup()


if __name__ == "__main__":
    exercise()
