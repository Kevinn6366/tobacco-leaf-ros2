"""Diagnostic only: three loosely aligned leaves test whether the jaw can retain a bundle.
This small fixture cannot satisfy the 24-leaf scene acceptance by itself.
"""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E
import numpy as np
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_multileaf_bench');dst.mkdir(exist_ok=True)
for name in ['runtime.py','feeding.py','stereo.py','orientation.py','height_regions.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:
 shutil.copy2(src/name,dst/name)
p=dst/'feeding.py';p.write_text(p.read_text().replace('floor+.004','floor+.001'))
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='40'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_multileaf_bench'"))
r=E.parse(src/'world.sdf');w=r.getroot().find('world');rng=np.random.default_rng(718);names=['leaf_00','leaf_01','leaf_02']
for m in list(w.findall('model')):
 if not(m.get('name').startswith('ground') or m.get('name') in names+['scale','box1']):w.remove(m)
for i,name in enumerate(names):
 x,y=rng.normal([.50,.25],[.006,.006]);yaw=rng.uniform(-.15,.15);w.find(f"model[@name='{name}']/pose").text=f'{x} {y} {.006+i*.006} 0 0 {yaw}'
r.write(dst/'world.sdf',encoding='unicode');manifest=json.loads((src/'scene_manifest.json').read_text());(dst/'scene_manifest.json').write_text(json.dumps({n:manifest[n] for n in names}));(dst/'evidence').mkdir(exist_ok=True)
(dst/'bench.py').write_text('''import rclpy,json
from runtime import Robot,P
from feeding import feed
rclpy.init();r=Robot()
try:
 feed(r,35,5)
 result=dict(evidence_type='three-leaf fixture diagnostic, not the 24-leaf main acceptance',complete=r.results.get('complete',False),multi_retained=any(c.get('outcome')=='deposited' and c.get('wrist_mass_g',0)>16 for c in r.results.get('cycles',[])),batch=r.results)
 (P/'evidence/result.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in result.items() if k!='batch'},flush=True)
finally:rclpy.shutdown();r.thread.join(3)
''')
print(dst)
