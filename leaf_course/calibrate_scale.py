"""Separate-world calibration with physical known masses; never used as a runtime mass oracle."""
import subprocess,os,time,re,json,xml.etree.ElementTree as E
from pathlib import Path
P=Path(__file__).resolve().parent
rows=[]
for mass in [0.,.05,.1,.2]:
 root=E.parse(P/'world.sdf').getroot();w=root.find('world');w.set('name','scale_calibration')
 for m in list(w.findall('model')):
  if not (m.get('name').startswith('ground') or m.get('name') in ['scale','box1']):w.remove(m)
 for pl in list(w.findall('plugin')):
  if pl.get('name').endswith('Sensors'):w.remove(pl)
 if mass:
  m=E.SubElement(w,'model',name='calibration_mass');E.SubElement(m,'pose').text='.49 -.22 .07 0 0 0';l=E.SubElement(m,'link',name='body');i=E.SubElement(l,'inertial');E.SubElement(i,'mass').text=str(mass);ii=E.SubElement(i,'inertia')
  for name in ['ixx','iyy','izz']:E.SubElement(ii,name).text=str(mass*.05*.05/6)
  for name in ['ixy','ixz','iyz']:E.SubElement(ii,name).text='0'
  co=E.SubElement(l,'collision',name='weight');E.SubElement(E.SubElement(E.SubElement(co,'geometry'),'box'),'size').text='.05 .05 .05'
 path=P/'scale-calibration-world.sdf';E.ElementTree(root).write(path,encoding='unicode')
 env=os.environ.copy();env['GZ_PARTITION']='leaf_scale_calibration';log=open(P/'scale-calibration.log','a')
 proc=subprocess.Popen(['gz','sim','-s','-r','-v','1','--physics-engine','gz-physics-bullet-featherstone-plugin',str(path)],env=env,stdout=log,stderr=log,start_new_session=True)
 try:
  time.sleep(6)
  raw=subprocess.check_output(['gz','topic','-e','-t','/scale/joints','-n','20'],env=env,text=True,timeout=15)
  vals=[float(x) for x in re.findall(r'^\s+position: (-?[\d.eE+-]+)$',raw,re.M)]
  assert vals,raw[:1000]
  row={'known_added_kg':mass,'spring_force_N':-1500*sum(vals)/len(vals),'positions':vals};rows.append(row);print(row,flush=True)
 finally:
  import signal
  os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=3)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
(P/'evidence'/'scale-calibration.json').write_text(json.dumps(rows,indent=2))
