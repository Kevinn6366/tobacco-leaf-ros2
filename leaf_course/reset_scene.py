"""Full reset by restarting the isolated world; preserve all historical evidence."""
from pathlib import Path
import subprocess,tarfile,datetime,fcntl,json,os
P=Path(__file__).resolve().parent
lock=open(P/'reset.lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('Reset already in progress')
subprocess.run(['bash',str(P/'stop.sh')],cwd=P,check=True)
stamp=datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f');history=P/'history';history.mkdir(exist_ok=True)
items=[]
for pattern in ['pile-*','box-*','scene-start-*','scene-settled-*']:items.extend((P/'evidence').glob(pattern))
items += [P/'evidence/batch-result.json',P/'evidence/scale-trace.json',P/'status.json']
with tarfile.open(history/(stamp+'.tar.gz'),'w:gz') as archive:
 for item in items:
  if item.is_file():archive.add(item,arcname=str(item.relative_to(P)))
# Archive first, then remove stale per-run outputs so new trials cannot inherit them.
for item in items:
 if item.is_file():item.unlink()
(P/'evidence/batch-result.json').write_text(json.dumps({'complete':False,'reset':True,'delivered_g':0.,'cycles':[]}))
(P/'fresh_session.json').write_text(json.dumps({'reset_at':stamp,'empty_box_required':True}))
subprocess.run(['bash',str(P/'start.sh')],cwd=P,check=True)
