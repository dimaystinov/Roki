"""Ограничиваем поиск GStreamer библиотеками внутри поставки."""
import os
import sys
from pathlib import Path
base = Path(sys._MEIPASS)
if sys.platform == 'win32':
    # Qt 6.7.3 в Windows-поставке падает при JIT-компиляции полного Operator.qml.
    # Интерпретатор без дискового QML-кэша проверен на полном интерфейсе в CI.
    os.environ['QV4_FORCE_INTERPRETER'] = '1'
    os.environ['QML_DISABLE_DISK_CACHE'] = '1'
    os.environ.setdefault('QT_QUICK_CONTROLS_STYLE', 'Fusion')
    os.environ.setdefault('QT_QUICK_BACKEND', 'software')
os.environ['GST_PLUGIN_SYSTEM_PATH_1_0'] = str(base/'gst_plugins')
os.environ['GST_PLUGIN_PATH_1_0'] = str(base/'gst_plugins')
os.environ['GST_REGISTRY_FORK'] = 'no'
# Реестр отдельный от системного GStreamer, пути обновляются при переносе приложения.
cache = Path.home()/'.cache'/'fpv-roki'
cache.mkdir(parents=True, exist_ok=True)
os.environ['GST_REGISTRY_1_0'] = str(cache/'gstreamer-registry.bin')
