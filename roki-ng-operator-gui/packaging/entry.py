"""Запуск автономного FPV Roki; подключение выполняется только оператором."""
import sys
from pathlib import Path
if sys.platform == 'win32' and any(a.startswith('--self-test-') for a in sys.argv):
    # У оконного exe нет консоли: сохраняем диагностику CI и исключаем диалог ошибки.
    import os
    import traceback
    sys.stdout = sys.stderr = open('fpv-self-test.log', 'w', encoding='utf-8', buffering=1)
    import faulthandler
    faulthandler.enable(file=sys.stderr, all_threads=True)
    print('SELF_TEST_START', sys.argv, flush=True)
    def test_exception(kind, value, tb):
        traceback.print_exception(kind, value, tb)
        sys.stderr.flush()
        os._exit(1)
    sys.excepthook = test_exception
import roki_operator
from PySide6.QtCore import QTimer
if sys.platform == 'win32' and any(a.startswith('--self-test-') for a in sys.argv):
    from PySide6.QtCore import qInstallMessageHandler
    qInstallMessageHandler(lambda kind, context, message: print('QT:', message, flush=True))
from PySide6.QtGui import QGuiApplication

# Изолированные проверки загрузки QML для диагностики нативных сбоев на Windows.
probe = next((a for a in sys.argv if a.startswith('--self-test-qml-')), None)
if probe:
    from PySide6.QtQml import QQmlApplicationEngine
    app = QGuiApplication([])
    engine = QQmlApplicationEngine()
    if probe.endswith('docking'):
        from operator_gui.native_docking import configure
        configure(engine, Path(sys._MEIPASS))
        qml = '''import QtQuick
import com.kdab.dockwidgets 2.0 as KDDW
Window { visible: true; width: 640; height: 480
    KDDW.DockingArea { anchors.fill: parent; uniqueName: "probe-area"
        KDDW.DockWidget { id: dock; uniqueName: "probe"; Rectangle { color: "white" } }
        Component.onCompleted: addDockWidget(dock, KDDW.KDDockWidgets.Location_OnLeft)
    }
}'''
    elif probe.endswith('controls'):
        qml = 'import QtQuick; import QtQuick.Controls; ApplicationWindow { visible: true; width: 320; height: 240; Button { text: "FPV Roki" } }'
    else:
        qml = 'import QtQuick; Window { visible: true; width: 320; height: 240 }'
    print('QML_PROBE_LOAD', probe, flush=True)
    engine.loadData(qml.encode())
    if not engine.rootObjects():
        raise SystemExit(1)
    print('QML_PROBE_OK', probe, flush=True)
    QTimer.singleShot(500, app.quit)
    raise SystemExit(app.exec())

if '--self-test-video' in sys.argv:
    import runpy
    runpy.run_path(str(Path(sys._MEIPASS)/'checks/check_gstreamer.py'), run_name='__main__')
    raise SystemExit(0)
gui_test = '--self-test-gui' in sys.argv
if gui_test:
    sys.argv.remove('--self-test-gui')
original = roki_operator.create_engine
def create_engine(controller):
    engine = original(controller)
    def brand():
        QGuiApplication.setApplicationDisplayName('FPV Roki')
        for window in engine.rootObjects():
            window.setProperty('title', 'FPV Roki')
        if gui_test:
            print('GUI_SELF_TEST_OK', flush=True)
            QTimer.singleShot(500, QGuiApplication.instance().quit)
    QTimer.singleShot(0, brand)
    return engine
roki_operator.create_engine = create_engine
if '--robot' not in sys.argv:
    sys.argv += ['--robot', '192.168.0.244']
raise SystemExit(roki_operator.main())
