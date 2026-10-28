"""Prepare isolated fixed-center single-leaf physical grasp test (ROS domain 35).
Run desktop_exec.py sim then desktop_exec.py \"python3 bench.py\" inside /tmp/leaf_flex_bench.
This component test bypasses perception only to isolate material/contact behavior.
"""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_flex_bench');dst.mkdir(exist_ok=True)
for name in ['runtime.py','stereo.py','orientation.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:
 shutil.copy2(src/name,dst/name)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='35'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_flex_bench'"))
root=E.parse(src/'world.sdf');w=root.getroot().find('world')
for m in list(w.findall('model')):
 if not (m.get('name').startswith('ground') or m.get('name') in ['source_region','leaf_00']):w.remove(m)
leaf=w.find("model[@name='leaf_00']");leaf.find('pose').text='.50 .25 .006 0 0 0'
# Component test keeps the same material and contact parameters as the live scene.
root.write(dst/'world.sdf',encoding='unicode');manifest=json.loads((src/'scene_manifest.json').read_text());manifest={'leaf_00':manifest['leaf_00']};(dst/'scene_manifest.json').write_text(json.dumps(manifest));(dst/'evidence').mkdir(exist_ok=True)
(dst/'bench.py').write_text('''import rclpy,json,numpy as np
from runtime import Robot,P
rclpy.init();r=Robot();result={}
try:
 r.wait_ready();r.hand(False);r.move([.38,0,.55]);r.hand(True,1.2);zero,_=r.sample();axis=zero/np.linalg.norm(zero);r.hand(False)
 r.move([.50,.25,.45]);r.sleep(.5);r.sense();r.snapshot('bench-before')
 r.move([.50,.25,.1086],duration=5,linear=True);r.hand(True,1.2,stiffness=250,closed_position=0.);r.snapshot('bench-closed');result['contacts']=r.contact_names()
 r.move([.50,.25,.45],duration=6,linear=True);r.sleep(.5);r.snapshot('bench-lift');result['lowest_leaf_m']=min(p['xyz'][2] for p in r.truth()['leaf_00'])
 r.move([.38,0,.55],duration=4);w,std=r.sample();result.update(wrist_g=float(np.dot(w-zero,axis)*1000/9.8),force_std_N=std.tolist(),final_leaf=r.truth()['leaf_00']);r.snapshot('bench-weighed')
except Exception as e:result['error']=str(e)
finally:
 (P/'evidence/result.json').write_text(json.dumps(result,indent=2));print(result,flush=True);rclpy.shutdown();r.thread.join(3)
''')
print(dst)
