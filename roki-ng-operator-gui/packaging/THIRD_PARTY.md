# Компоненты поставки FPV Roki

Интерфейс основан на https://github.com/aleksey-yagubov/roki-ng-operator-gui .
Пакет содержит Python, PySide6/Shiboken6, Qt, KDDockWidgets, PyGObject,
GLib/GObject/GIRepository, GStreamer и плагины, FFmpeg/libav, NumPy и MessagePack.
Локальный тест H.264 использует x264. Это не собственные компоненты FPV Roki;
применяются лицензии соответствующих авторов, в том числе LGPL и GPL.

Исходники:
- https://www.python.org/downloads/source/
- https://code.qt.io/
- https://github.com/KDAB/KDDockWidgets (v2.4.1; архив приложен в проекте)
- https://gitlab.freedesktop.org/gstreamer/gstreamer
- https://gitlab.gnome.org/GNOME/pygobject
- https://gitlab.gnome.org/GNOME/glib
- https://ffmpeg.org/download.html
- https://code.videolan.org/videolan/x264
- https://github.com/numpy/numpy
- https://github.com/msgpack/msgpack-python

Сборочные сценарии и конфигурация зависимостей находятся в packaging/ исходного
проекта. Сторонние библиотеки поставляются отдельными динамическими файлами,
а не статически встраиваются в код интерфейса.
