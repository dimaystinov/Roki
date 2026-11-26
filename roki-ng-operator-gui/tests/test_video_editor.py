"""Run QML in its own process: the other tests use QCoreApplication."""
import os
from pathlib import Path
import subprocess
import sys


def test_video_editor_drafts_survive_polling():
    result = subprocess.run([sys.executable, str(Path(__file__).resolve()), '--qml'],
                            env=dict(os.environ, QT_QPA_PLATFORM='offscreen'),
                            capture_output=True, text=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr


def exercise_qml():
    from PySide6.QtCore import QObject, QMetaObject, Q_ARG, Qt, QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlComponent, QQmlEngine, QQmlExpression
    from PySide6.QtQuick import QQuickWindow
    from PySide6.QtTest import QTest
    app = QGuiApplication([])
    engine = QQmlEngine()
    component = QQmlComponent(engine, QUrl.fromLocalFile(str(
        Path(__file__).resolve().parents[1] / 'qml/ValueEditor.qml')))
    assert not component.isError(), component.errors()
    editor = component.createWithInitialProperties(dict(
        meta={'type':'int'}, initialValue=30, preserveDraft=True))
    assert editor is not None, component.errors()
    window = QQuickWindow()
    window.resize(400, 100)
    editor.setParentItem(window.contentItem())
    editor.setWidth(350)
    window.show()
    number = editor.findChild(QObject, 'valueInput')
    number.forceActiveFocus()
    QTest.keyClick(window, Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier)
    QTest.keyClick(window, Qt.Key.Key_5)
    QTest.keyClick(window, Qt.Key.Key_9)
    assert editor.property('dirty') is True
    for value in (30, 25, 30):
        editor.setProperty('initialValue', value)
        editor.setProperty('meta', {'type':'int', 'min':1, 'max':120})
        QTest.qWait(20)
        assert number.property('text') == '59'
        assert number.hasActiveFocus()
    def js(code):
        expr = QQmlExpression(engine.rootContext(), editor, code)
        result = expr.evaluate()
        assert not expr.hasError(), expr.error().toString()
        return result
    js('resetValue(59)')
    assert editor.property('dirty') is False
    editor.setProperty('initialValue', 24)
    assert number.property('text') == '24'
    # Choice metadata is rebuilt by every catalog response too.
    editor.setProperty('meta', {'type':'str', 'choices':['one','two']})
    editor.setProperty('initialValue', 'one')
    QTest.qWait(20)
    choice = editor.findChild(QObject, 'valueChoice')
    choice.setProperty('currentIndex', 1)
    QMetaObject.invokeMethod(choice, 'activated', Q_ARG(int, 1))
    editor.setProperty('meta', {'type':'str', 'choices':['one','two'], 'live':True})
    editor.setProperty('initialValue', 'one')
    QTest.qWait(20)
    assert editor.property('value') == 'two'
    js('reset()')  # Stream selection explicitly discards the old stream's draft.
    assert editor.property('value') == 'one'
    window.close()
    editor.setParentItem(None)
    editor.deleteLater()
    app.processEvents()


if __name__ == '__main__':
    exercise_qml()
