#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
python3 - <<'PY'
import os,signal,time
from pathlib import Path
for name in ('panel','rviz','sim'):
 p=Path(name+'.pid')
 if p.exists():
  pid=int(p.read_text())
  try:
   cmd=Path(f'/proc/{pid}/cmdline').read_bytes().decode(errors='replace')
   expected={'panel':'app.py','rviz':'rviz2','sim':'course.launch.py'}[name]
   if expected in cmd and Path(f'/proc/{pid}/cwd').resolve()==Path.cwd() and os.getpgid(pid)==pid:
    os.killpg(pid,signal.SIGINT if name=='sim' else signal.SIGTERM)
    for _ in range(30):
     if not Path(f'/proc/{pid}').exists():break
     time.sleep(.1)
    if Path(f'/proc/{pid}').exists():os.killpg(pid,signal.SIGKILL)
  except (FileNotFoundError,ProcessLookupError):pass
PY
