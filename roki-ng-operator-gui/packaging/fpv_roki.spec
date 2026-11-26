from pathlib import Path
import sys
root = Path(SPECPATH).parent
mac = sys.platform == 'darwin'
platform_dir = 'macos' if mac else 'windows'
bootstrap = root/'.deps'/platform_dir/'bootstrap'
libs = list(bootstrap.glob('**/*roki_docking_bootstrap.' + ('dylib' if mac else 'dll')))
if not libs:
    raise RuntimeError('Build native docking bootstrap first')
binaries = [(str(libs[0]), 'docking')]
datas = [(str(root/'qml'), 'qml'),
         (str(root/'packaging/licenses'), 'licenses/runtime'),
         (str(root/'packaging/THIRD_PARTY.md'), 'licenses'),
         (str(root/'scripts/check_gstreamer.py'), 'checks'),
         (str(root/'.deps'/platform_dir/'qt-version.txt'), 'docking'),
         (str(root/'native/licenses'), 'licenses')]
a = Analysis([str(root/'packaging/entry.py')], pathex=[str(root)],
    binaries=binaries, datas=datas,
    hiddenimports=['gi.repository.Gst', 'gi.repository.GstVideo', 'gi.repository.Gio'],
    runtime_hooks=[str(root/'packaging/frozen_runtime.py')],
    hooksconfig={'gstreamer': {'include_plugins': ['coreelements','app','udp','rtp','rtpmanager','videoparsersbad','videoconvertscale','libav','videotestsrc','openh264','x264']}, 'gi': {'languages': ['ru','en']}},
    excludes=['PyQt5','PyQt6','tkinter','matplotlib','pandas','IPython'])
# KDDockWidgets собран с Homebrew Qt: нельзя смешивать его с Qt из wheel.
if mac:
    import subprocess
    qt = Path(subprocess.check_output(['brew', '--prefix', 'qt'], text=True).strip())
    def same_qt(toc):
        result = []
        for dest, source, kind in toc:
            marker = '/site-packages/PySide6/Qt/'
            if marker in source:
                suffix = source.split(marker, 1)[1]
                candidate = qt/suffix if suffix.startswith('lib/') else qt/'share/qt'/suffix
                if candidate.is_file():
                    source = str(candidate)
                elif suffix.startswith(('plugins/', 'qml/')):
                    continue
                elif kind in ('BINARY', 'EXTENSION'):
                    raise RuntimeError(f'Missing matching Qt binary: {candidate}')
            result.append((dest, source, kind))
        return result
    a.binaries = same_qt(a.binaries)
    a.datas = same_qt(a.datas)
pyz = PYZ(a.pure)
exe = EXE(pyz,a.scripts,[],exclude_binaries=True,name='FPV Roki',console=False, debug=False)
coll = COLLECT(exe,a.binaries,a.datas,name='FPV Roki')
if mac:
    app = BUNDLE(coll,name='FPV Roki.app',icon=str(root/'packaging/AppIcon.icns'),bundle_identifier='ru.roki.fpv-operator',
        info_plist={'CFBundleDisplayName':'FPV Roki','LSMinimumSystemVersion':'26.0','NSHighResolutionCapable':True,
                    'NSLocalNetworkUsageDescription':'Управление Roki и приём видеопотока.'})
