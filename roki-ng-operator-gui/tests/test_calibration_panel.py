"""The same robot-described test card works for hardware and the simulator."""
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
    import roki_operator  # Apply the same Windows Qt interpreter settings as the app.
    import tempfile
    from PySide6.QtCore import QMetaObject, QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlComponent, QQmlEngine
    from PySide6.QtQuick import QQuickItem
    from operator_gui.controller import Controller
    from tests.fake_robot import FakeRobot
    from tests.test_operator import wait_until

    key = "motion.run_10_mm"

    class Robot(FakeRobot):
        def _result(self, op, body):
            if op == "test.list":
                return {"items": ["measured_walk"]}
            if op == "test.describe":
                return dict(name="measured_walk", title="Measured walk",
                            parameters={"cycles": dict(type="int", min=1, max=20, default=7)})
            if op == "params.describe" and body["key"] == key:
                return dict(key=key, type="float", min=1, max=10000,
                            description="Измеренный путь, мм")
            if op == "test.start":
                assert body == dict(name="measured_walk", cycles=7)
                self.jobs[1] = dict(job_id=1, status="completed", operation=op, progress=7,
                                    test={"name":"measured_walk"}, manual_parameters=[key])
                return dict(accepted=True, job_id=1)
            if op == "test.measure":
                assert body == dict(job_id=1, values={key: 330.})
                self.values.update(body["values"])
                return dict(job_id=1, saved=body["values"])
            return super()._result(op, body)

    def child(item, name):
        if item.objectName() == name:
            return item
        for sub in item.childItems():
            found = child(sub, name)
            if found is not None:
                return found

    app = QGuiApplication([])
    for robot_id in ("ROKI-TEST", "SIM-ROKI2"):
        robot = Robot(robot_id=robot_id)
        temp = tempfile.TemporaryDirectory()
        c = Controller("127.0.0.1", robot.port, Path(temp.name))
        engine = QQmlEngine()
        for name, obj in [("backend",c), ("controls",c.control), ("testsModel",c.tests), ("slotsModel",c.slots)]:
            engine.rootContext().setContextProperty(name,obj)
        component = QQmlComponent(engine, QUrl.fromLocalFile(str(Path(__file__).resolve().parents[1]/"qml/CatalogPanel.qml")))
        panel = component.createWithInitialProperties(dict(kind="tests"))
        try:
            assert isinstance(panel,QQuickItem), component.errors()
            panel.setWidth(640)
            panel.setHeight(640)
            c.connectRobot("127.0.0.1",robot.port)
            wait_until(lambda:c.transport.connected)
            c.control.acquire()
            wait_until(lambda:c.control.owns)
            c.control.enterManual()
            wait_until(lambda:c.control.view["ready"])
            c.requestCatalog("tests")
            wait_until(lambda:child(panel,"startTest_measured_walk") is not None and bool(c.test_schemas))
            assert QMetaObject.invokeMethod(child(panel,"startTest_measured_walk"),"clicked")
            wait_until(lambda:c.control.job.get("status")=="completed")
            wait_until(lambda:key in c.testMeasurements.get("1",{}))
            field=child(panel,"testMeasurement_"+key)
            assert field is not None
            field.setProperty("text","330")
            assert QMetaObject.invokeMethod(field,"textEdited")
            button=child(panel,"saveTestMeasurement_measured_walk")
            wait_until(button.isEnabled)
            assert QMetaObject.invokeMethod(button,"clicked")
            wait_until(lambda:robot.values.get(key)==330.)
            assert not any(m["op"].startswith(("sim.","camera.","detection.","videostream.")) for m in robot.requests)
        finally:
            c.shutdown()
            panel.deleteLater()
            app.processEvents()
            robot.close()

if __name__=="__main__":
    exercise()
