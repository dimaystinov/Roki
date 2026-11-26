"""Exercise test progress/result UI without the suite's QCoreApplication."""
import os
from pathlib import Path
import subprocess
import sys


def test_card_progress_and_terminal_results():
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--qml'],
                            env=dict(os.environ, QT_QPA_PLATFORM='offscreen'),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def exercise_qml():
    from PySide6.QtCore import QObject, Property, Signal, QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlComponent, QQmlEngine

    class Controls(QObject):
        changed = Signal()
        job = {}

        @Property('QVariantMap', notify=changed)
        def view(self):
            return dict(job=self.job, ready=True)

        def publish(self, job):
            self.job = job
            self.changed.emit()

    app = QGuiApplication([])
    engine = QQmlEngine()
    controls = Controls()
    engine.rootContext().setContextProperty('controls', controls)
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(
        Path(__file__).resolve().parents[1] / 'qml/TestCard.qml')))
    assert not component.isError(), component.errors()
    card = component.createWithInitialProperties(dict(testName='run_test', raw='', schema={'parameters': {}}))
    assert card is not None, component.errors()
    bar = card.findChild(QObject, 'testProgress_run_test')
    label = card.findChild(QObject, 'testResult_run_test')
    assert not bar.property('visible')
    job = dict(operation='test.start', job_id=1, status='running', progress=0,
               test=dict(name='run_test', progress_max=11))
    controls.publish(job)
    assert bar.property('visible')
    controls.publish(job | {'progress': 4})
    assert bar.property('value') == 4 and bar.property('to') == 11
    controls.publish({k: v for k, v in job.items() if k != 'progress'})
    assert not bar.property('visible')
    controls.publish(job | {'test': {'name': 'run_test'}})
    assert not bar.property('visible')
    for status, reason, text in [('completed', None, 'Успешно'),
                                 ('failed', 'Body fell', 'Ошибка: Body fell'),
                                 ('cancelled', None, 'Отменён')]:
        controls.publish(job | dict(status=status, reason=reason))
        assert not bar.property('visible')
        assert label.property('text') == text
    controls.publish(dict(operation='motion.drive', job_id=2, status='running', progress=0))
    assert label.property('text') == 'Отменён'
    controls.publish(job | dict(job_id=3, progress=0))
    assert bar.property('visible') and bar.property('value') == 0
    card.deleteLater()
    app.processEvents()


if __name__ == '__main__':
    exercise_qml()
