"""Isolated Yagubov GUI snapshots; never replaces Roki or connects to hardware."""
import hashlib
import importlib.abc
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import ssl
import certifi
import subprocess
import sys
import tempfile
import urllib.request
import zipfile

REPO = 'aleksey-yagubov/roki-ng-operator-gui'
LIMIT = 150 * 1024 * 1024


def home():
    if sys.platform == 'win32':
        base = Path(os.environ.get('LOCALAPPDATA', str(Path.home()/'AppData/Local')))
    elif sys.platform == 'darwin':
        base = Path.home()/'Library/Application Support'
    else:
        base = Path(os.environ.get('XDG_DATA_HOME', str(Path.home()/'.local/share')))
    return base/'Roki-Yagubov-Operator'


def tls_context():
    # Include Mozilla roots so frozen builds do not depend on Homebrew/conda
    # certificate paths from the developer machine. Retain OS trust as well.
    context = ssl.create_default_context()
    context.load_verify_locations(cafile=certifi.where())
    return context


def get(url):
    request=urllib.request.Request(url,headers={'User-Agent':'Roki-Upstream-Updater','Accept':'application/vnd.github+json'})
    with urllib.request.urlopen(request,timeout=60,context=tls_context()) as response:
        data=response.read(LIMIT+1)
    if len(data)>LIMIT:raise ValueError('Download exceeds 150 MiB')
    return data


def latest():
    metadata=json.loads(get('https://api.github.com/repos/'+REPO))
    branch=metadata['default_branch']
    from urllib.parse import quote
    commit=json.loads(get('https://api.github.com/repos/'+REPO+'/commits/'+quote(branch,safe='')))
    sha=commit['sha']
    if not re.fullmatch('[0-9a-f]{40}',sha):raise ValueError('Invalid GitHub commit')
    return {'repository':REPO,'branch':branch,'sha':sha,'message':commit['commit']['message'].splitlines()[0]}


def extract_source(data,destination):
    destination=Path(destination)
    with zipfile.ZipFile(io.BytesIO(data)) as z:
        if sum(i.file_size for i in z.infolist())>LIMIT*3:raise ValueError('Unpacked source too large')
        roots=set()
        # Upstream ships Linux ELF libraries/symlinks. Our native adapter uses
        # bundled platform libraries; never extract the Linux native subtree.
        entries=[i for i in z.infolist() if len(PurePosixPath(i.filename).parts)<2 or PurePosixPath(i.filename).parts[1]!='native']
        for info in entries:
            p=PurePosixPath(info.filename)
            if p.is_absolute() or '..' in p.parts or '\\' in info.filename or ':' in info.filename or stat.S_ISLNK(info.external_attr>>16):
                raise ValueError('Unsafe archive path: '+info.filename)
            if p.parts:roots.add(p.parts[0])
        if len(roots)!=1:raise ValueError('Expected one archive root')
        destination.mkdir(parents=True,exist_ok=False)
        for info in entries:
            p=PurePosixPath(info.filename)
            if len(p.parts)<2:continue
            target=destination.joinpath(*p.parts[1:])
            if info.is_dir():target.mkdir(parents=True,exist_ok=True)
            else:
                target.parent.mkdir(parents=True,exist_ok=True)
                with z.open(info) as source,target.open('wb') as out:shutil.copyfileobj(source,out)
    for required in ['roki_operator.py','operator_gui/__init__.py','qml/Operator.qml']:
        if not (destination/required).is_file():raise ValueError('Missing '+required)
    for p in destination.rglob('*.py'):compile(p.read_bytes(),str(p),'exec')


def pointer(root):
    p=Path(root)/'current.json'
    return json.loads(p.read_text()) if p.exists() else {}


def write_pointer(root,value):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    p=root/'current.json.tmp'
    with p.open('w',encoding='utf-8') as f:
        json.dump(value,f);f.flush();os.fsync(f.fileno())
    p.replace(root/'current.json')


def activate(root,sha,probe):
    if not re.fullmatch('[0-9a-f]{40}',sha):raise ValueError('Invalid commit')
    version=Path(root)/'versions'/sha
    if not version.is_dir():raise ValueError('Snapshot is absent')
    probe(version)
    old=pointer(root)
    if old.get('sha')!=sha:write_pointer(root,{'sha':sha,'previous':old.get('sha')})


def rollback(root):
    current=pointer(root);previous=current.get('previous')
    if not previous or not (Path(root)/'versions'/previous).is_dir():raise ValueError('No previous installation')
    write_pointer(root,{'sha':previous,'previous':current['sha']})


class SourceFinder(importlib.abc.MetaPathFinder):
    """Do not let PyInstaller silently substitute its own operator_gui modules."""
    def __init__(self,root):self.root=Path(root).resolve()
    def find_spec(self,fullname,path=None,target=None):
        if fullname!='roki_operator' and fullname!='operator_gui' and not fullname.startswith('operator_gui.'):
            return None
        base=self.root.joinpath(*fullname.split('.'))
        package=base/'__init__.py';module=base.with_suffix('.py')
        if package.is_file():return importlib.util.spec_from_file_location(fullname,package,submodule_search_locations=[str(base)])
        if module.is_file():return importlib.util.spec_from_file_location(fullname,module)
        raise ModuleNotFoundError('Upstream module missing: '+fullname)


def command(*args):
    if getattr(sys,'frozen',False):return [sys.executable,*args]
    return [sys.executable,str(Path(__file__).resolve()),*args]


def probe(version):
    with tempfile.TemporaryDirectory(prefix='roki-upstream-probe-') as config:
        log=Path(config)/'probe.log'
        env={**os.environ,'ROKI_UPSTREAM_LOG':str(log)}
        result=subprocess.run(command('--upstream-source',str(version),'--config-dir',config,'--check-gui'),
            capture_output=True,text=True,timeout=60,env=env)
        output=result.stdout+result.stderr+(log.read_text(encoding='utf-8',errors='replace') if log.exists() else '')
        if result.returncode or 'UPSTREAM_GUI_OK' not in output:
            raise RuntimeError('Проверка запуска не пройдена; прежняя версия сохранена.\n'+output[-5000:])


def update(root,progress=print):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    # Exclusive lock is released on normal success/failure, never silently stolen.
    lock=root/'update.lock'
    fd=os.open(lock,os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    try:
        os.close(fd)
        info=latest();sha=info['sha'];progress('Коммит '+sha[:12]+': '+info['message'])
        version=root/'versions'/sha
        if not version.exists():
            progress('Скачивание исходников '+REPO)
            data=get('https://api.github.com/repos/'+REPO+'/zipball/'+sha)
            with tempfile.TemporaryDirectory(dir=root,prefix='stage-') as tmp:
                source=Path(tmp)/'source';extract_source(data,source)
                info['archive_sha256']=hashlib.sha256(data).hexdigest()
                (source/'roki-updater-manifest.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
                version.parent.mkdir(parents=True,exist_ok=True);source.rename(version)
        progress('Проверка Python, QML и нативного модуля…')
        activate(root,sha,probe)
        progress('Установлено отдельно: '+sha[:12]+'\n'+str(version))
        return info
    finally:lock.unlink(missing_ok=True)


def run_source(source,args):
    source=Path(source).resolve()
    # Portability adapter only; all operator logic and QML come from upstream.
    from operator_gui import native_docking
    import types
    platform_adapter=types.ModuleType('operator_gui.native_docking')
    runtime_root=Path(__file__).resolve().parent
    platform_adapter.configure=lambda engine,root:native_docking.configure(engine,runtime_root)
    for name in list(sys.modules):
        if name=='roki_operator' or name=='operator_gui' or name.startswith('operator_gui.'):
            del sys.modules[name]
    sys.meta_path.insert(0,SourceFinder(source))
    sys.path.insert(0,str(source))
    sys.modules['operator_gui.native_docking']=platform_adapter
    import roki_operator
    if Path(roki_operator.__file__).resolve()!=source/'roki_operator.py':raise RuntimeError('Wrong source module')
    sys.argv=[str(source/'roki_operator.py'),*args]
    if '--config-dir' not in args:sys.argv+=['--config-dir',str(home()/'config')]
    check='--check-gui' in args
    if check:
        sys.argv.remove('--check-gui')
        original=roki_operator.create_engine
        def checked(controller):
            engine=original(controller)
            from PySide6.QtCore import QTimer
            def confirm():
                from PySide6.QtGui import QGuiApplication
                if engine.rootObjects():
                    print('UPSTREAM_GUI_OK',flush=True)
                    QGuiApplication.instance().exit(0)
                else:QGuiApplication.instance().exit(1)
            QTimer.singleShot(100,confirm)
            return engine
        roki_operator.create_engine=checked
    return roki_operator.main()


def dialog():
    from PySide6.QtWidgets import QApplication,QWidget,QVBoxLayout,QLabel,QPushButton,QPlainTextEdit
    from PySide6.QtCore import QThread,Signal
    app=QApplication([]);window=QWidget();window.setWindowTitle('Operator NG — обновление версии Ягубова');window.resize(720,480)
    layout=QVBoxLayout(window)
    label=QLabel('Отдельная установка aleksey-yagubov/roki-ng-operator-gui.\nRoki и его настройки не изменяются. Роботный runtime обновляется отдельно.');label.setWordWrap(True);layout.addWidget(label)
    log=QPlainTextEdit();log.setReadOnly(True);layout.addWidget(log)
    buttons=[]
    class Worker(QThread):
        message=Signal(str)
        def run(self):
            try:update(home(),self.message.emit)
            except Exception as e:self.message.emit('Ошибка: '+str(e))
    worker=Worker(window);worker.message.connect(log.appendPlainText)
    def enable():
        for b in buttons:b.setEnabled(True)
    worker.finished.connect(enable)
    def install():
        for b in buttons:b.setEnabled(False)
        worker.start()
    def run():
        try:
            state=pointer(home());sha=state.get('sha')
            if not sha:raise ValueError('Сначала установите версию')
            subprocess.Popen(command('--upstream-run'))
        except Exception as e:log.appendPlainText(str(e))
    def undo():
        try:rollback(home());log.appendPlainText('Возврат к '+pointer(home())['sha'][:12])
        except Exception as e:log.appendPlainText(str(e))
    for text,fn in [('Установить / обновить версию Ягубова',install),('Запустить отдельную версию',run),('Вернуть предыдущую версию',undo)]:
        button=QPushButton(text);button.clicked.connect(fn);layout.addWidget(button);buttons.append(button)
    def close(event):
        if worker.isRunning():log.appendPlainText('Дождитесь завершения обновления.');event.ignore()
        else:event.accept()
    window.closeEvent=close
    state=pointer(home());log.appendPlainText('Текущий коммит: '+state.get('sha','не установлен'))
    window.show();return app.exec()


def main(args=None):
    args=list(sys.argv[1:] if args is None else args)
    if '--upstream-source' in args:
        i=args.index('--upstream-source');source=args[i+1];del args[i:i+2]
        return run_source(source,args)
    if '--upstream-run' in args:
        args.remove('--upstream-run');state=pointer(home());sha=state.get('sha','')
        if not re.fullmatch('[0-9a-f]{40}',sha):raise ValueError('Сначала установите версию через updater')
        return run_source(home()/'versions'/sha,args)
    if '--upstream-update-cli' in args:update(home());return 0
    if '--upstream-rollback' in args:rollback(home());return 0
    return dialog()

if __name__=='__main__':raise SystemExit(main())
