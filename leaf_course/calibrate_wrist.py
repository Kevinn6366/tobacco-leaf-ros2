"""Isolated physical-load calibration; never supplies runtime grasp weights."""
from pathlib import Path
import shutil,subprocess,time,json,os,signal,xml.etree.ElementTree as E
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_wrist_calibration');dst.mkdir(exist_ok=True)
for name in ['runtime.py','stereo.py','orientation.py','feeding.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:shutil.copy2(src/name,dst/name)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='34'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_wrist_calibration'"))
root=E.parse(src/'world.sdf');w=root.getroot().find('world')
for m in list(w.findall('model')):
 if not m.get('name').startswith('ground'):w.remove(m)
root.write(dst/'world.sdf',encoding='unicode');(dst/'scene_manifest.json').write_text('{}');(dst/'evidence').mkdir(exist_ok=True)
(dst/'measure.py').write_text('''import rclpy,json
from runtime import Robot,P
rclpy.init();r=Robot()
try:
 r.wait_ready();r.hand(False);r.move([.38,0,.55],sideways=False);r.hand(True,3.5);w,std=r.sample();(P/'measurement.json').write_text(json.dumps({'force_N':w.tolist(),'std_N':std.tolist(),'deflection_m':r.wrist_q}))
finally:rclpy.shutdown();r.thread.join(3)
''')
rows=[]
for mass in [0.,.05,.1,.2]:
 root=E.parse(src/'panda.sdf');el=root.find("model/link[@name='panda_hand']/inertial/mass");el.text=str(float(el.text)+mass);root.write(dst/'panda.sdf',encoding='unicode')
 result=dst/'measurement.json'
 if result.exists():result.unlink()
 subprocess.run(['python3','desktop_exec.py','sim'],cwd=dst,check=True)
 subprocess.run(['python3','desktop_exec.py','python3 measure.py','measure.log'],cwd=dst,check=True)
 try:
  end=time.monotonic()+120
  while not result.exists() and time.monotonic()<end:time.sleep(1)
  if not result.exists():raise RuntimeError('Measurement missing; inspect '+str(dst/'measure.log'))
  d=json.loads(result.read_text());d['known_added_kg']=mass;rows.append(d);print(d,flush=True)
 finally:subprocess.run(['bash','stop.sh'],cwd=dst,check=True)
(src/'evidence/wrist-calibration.json').write_text(json.dumps(rows,indent=2))
