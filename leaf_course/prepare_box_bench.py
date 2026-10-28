"""Isolated physical whole-box transport with a free 50 g calibration payload."""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_box_bench');dst.mkdir(exist_ok=True)
for name in ['runtime.py','feeding.py','stereo.py','orientation.py','height_regions.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:
 shutil.copy2(src/name,dst/name)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='39'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_box_bench'"))
r=E.parse(src/'world.sdf');w=r.getroot().find('world')
for m in list(w.findall('model')):
 if not(m.get('name').startswith('ground') or m.get('name') in ['scale','box1']):w.remove(m)
m=E.SubElement(w,'model',name='calibration_payload');E.SubElement(m,'pose').text='.49 -.22 .04 0 0 0';l=E.SubElement(m,'link',name='body');i=E.SubElement(l,'inertial');E.SubElement(i,'mass').text='.05';ii=E.SubElement(i,'inertia')
for key,value in [('ixx',.00002833),('iyy',.00002833),('izz',.00005333),('ixy',0),('ixz',0),('iyz',0)]:E.SubElement(ii,key).text=str(value)
for tag in ['visual','collision']:
 element=E.SubElement(l,tag,name=tag);E.SubElement(E.SubElement(E.SubElement(element,'geometry'),'box'),'size').text='.08 .08 .02'
 if tag=='visual':E.SubElement(E.SubElement(element,'material'),'diffuse').text='.8 .2 .1 1'
r.write(dst/'world.sdf',encoding='unicode');(dst/'scene_manifest.json').write_text('{}');(dst/'evidence').mkdir(exist_ok=True)
(dst/'bench.py').write_text('''import rclpy,json,subprocess,numpy as np
from runtime import Robot,P
from feeding import deliver_box
rclpy.init();r=Robot();result={'evidence_type':'physical box transport component test with free 50 g calibration payload; not a leaf feeding batch'}
try:
 r.wait_ready();r.sleep(2);r.hand(False,force=5.);r.move([.38,0,.55]);result['box_delivery']=deliver_box(r)
 raw=subprocess.check_output(['gz','topic','-e','-t','/world/leaf_course/pose/info','-n','1','--json-output'],text=True,timeout=10);(P/'evidence/world-final.json').write_text(raw)
 poses={x['name']:x for x in json.loads(raw)['pose']};a=poses['calibration_payload']['position'];b=poses['box1']['position'];rel=np.array([a[k]-b[k] for k in 'xyz']);result['payload_relative_xyz']=rel.tolist();result['payload_retained']=bool(abs(rel[0])<.14 and abs(rel[1])<.13 and 0<rel[2]<.1);result['pass']=result['payload_retained']
except Exception as e:result.update(error=str(e),pass_test=False)
finally:
 result['attempt']=r.results.get('box_delivery_attempt');(P/'evidence/result.json').write_text(json.dumps(result,indent=2));print(result,flush=True);rclpy.shutdown();r.thread.join(3)
''')
print(dst)
