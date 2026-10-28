"""Isolated center-grasp and physical-release bench; known point isolates contact mechanics."""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_release_bench');dst.mkdir(exist_ok=True)
for name in ['runtime.py','stereo.py','orientation.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:
 shutil.copy2(src/name,dst/name)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='36'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_release_bench'"))
r=E.parse(src/'world.sdf');w=r.getroot().find('world')
for m in list(w.findall('model')):
 if not(m.get('name').startswith('ground') or m.get('name') in ['leaf_00','scale','box1']):w.remove(m)
w.find("model[@name='leaf_00']/pose").text='.50 .25 .006 0 0 0';r.write(dst/'world.sdf',encoding='unicode')
m=json.loads((src/'scene_manifest.json').read_text());(dst/'scene_manifest.json').write_text(json.dumps({'leaf_00':m['leaf_00']}));(dst/'evidence').mkdir(exist_ok=True)
(dst/'bench.py').write_text('''import rclpy,json,numpy as np
from runtime import Robot,P
rclpy.init();r=Robot();result={}
try:
 r.wait_ready();r.hand(False);r.move([.38,0,.55]);r.hand(True,1.2);zero,_=r.sample();axis=zero/np.linalg.norm(zero);r.hand(False);scale0,_=r.scale_sample()
 r.move([.50,.25,.45]);r.sense();r.snapshot('before')
 r.move([.50,.25,.1124],duration=5,linear=True);r.hand(True,1.2,closed_position=0.);r.snapshot('closed')
 r.move([.50,.25,.45],duration=6,linear=True);r.move([.38,0,.55],duration=4);w,_=r.sample();result['wrist_g']=float((w-zero)@axis*1000/9.8);r.snapshot('held')
 b=r.box_xyz();drop=[float(b[0]),float(b[1]+.075),float(b[2]+.34)];r.move([drop[0],drop[1],.55],duration=4);r.move(drop);r.hand(False);r.sleep(1);r.snapshot('released')
 r.move([.30,-.02,.58]);scale,_=r.scale_sample();result['scale_gain_g']=(scale-scale0)*1000/9.8;r.snapshot('verified')
 result['pass']=result['wrist_g']>8 and abs(result['scale_gain_g']-result['wrist_g'])<1
except Exception as e:result['error']=str(e);result['pass']=False
finally:
 (P/'evidence/result.json').write_text(json.dumps(result,indent=2));print(result,flush=True);rclpy.shutdown();r.thread.join(3)
''')
print(dst)
