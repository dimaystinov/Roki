"""Проверка загрузочных зависимостей Mach-O: никаких путей машины сборки."""
from pathlib import Path
import sys
from macholib.MachO import MachO
root=Path(sys.argv[1]);seen=set();bad=[];count=0
for path in root.rglob('*'):
    if not path.is_file():continue
    real=path.resolve()
    if real in seen:continue
    seen.add(real)
    if not real.is_relative_to(root.resolve()):bad.append((str(path),'external symlink',str(real)));continue
    with real.open('rb') as f:magic=f.read(4)
    if magic not in (b'\xcf\xfa\xed\xfe',b'\xce\xfa\xed\xfe',b'\xfe\xed\xfa\xcf',b'\xca\xfe\xba\xbe'):continue
    count+=1
    for header in MachO(str(real)).headers:
        for cmd,info,data in header.commands:
            if cmd.get_cmd_name() not in ('LC_LOAD_DYLIB','LC_LOAD_WEAK_DYLIB','LC_REEXPORT_DYLIB','LC_RPATH'):continue
            name=data.split(b'\0',1)[0].decode()
            if name.startswith('/') and not name.startswith(('/System/Library/','/usr/lib/')):
                bad.append((str(path.relative_to(root)),cmd.get_cmd_name(),name))
print('Mach-O files:',count)
for item in bad:print(*item,sep=': ')
if bad:raise SystemExit(1)
print('No external non-system Mach-O dependencies or symlinks')
