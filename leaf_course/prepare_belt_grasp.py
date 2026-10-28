"""Prepare an isolated indexed-conveyor grasp test, keeping the user's heap intact."""
from pathlib import Path
import shutil,json,xml.etree.ElementTree as E
src=Path(__file__).resolve().parent;dst=Path('/tmp/leaf_belt_grasp');dst.mkdir(exist_ok=True)
for name in ['feeding.py','runtime.py','stereo.py','orientation.py','height_regions.py','motion_compensation.py','kinematics.py','perception.py','camera.json','panda.urdf','panda.sdf','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']:
 shutil.copy2(src/name,dst/name)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='38'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_belt_grasp'"))
# Same Bullet solver as the heap: a physical translating top belt segment.
# Joint velocity actuation moves the surface; leaves move only through contact.
r=E.parse(src/'world.sdf');w=r.getroot().find('world')
for m in list(w.findall('model')):
 if not(m.get('name').startswith('ground') or m.get('name') in ['leaf_00','scale','box1']):w.remove(m)
for ground in w.findall('model'):
 if ground.get('name').startswith('ground') and ground.get('name')!='ground_north':
  pose=ground.find('pose').text.split();pose[2]='-.10';ground.find('pose').text=' '.join(pose)
m=w.find("model[@name='ground_north']");m.set('name','conveyor')
m.find('static').text='false';m.find('pose').text='.49 .25 -.025 0 0 0'
l=m.find('link')
for size in l.findall('.//box/size'):size.text='.34 .55 .05'
inertial=E.SubElement(l,'inertial');E.SubElement(inertial,'mass').text='2';inertia=E.SubElement(inertial,'inertia')
for tag,value in [('ixx',.0508),('iyy',.0197),('izz',.0697),('ixy',0),('ixz',0),('iyz',0)]:E.SubElement(inertia,tag).text=str(value)
base=E.SubElement(m,'link',name='base');bi=E.SubElement(base,'inertial');E.SubElement(bi,'mass').text='1';ii=E.SubElement(bi,'inertia')
for tag,value in [('ixx',.01),('iyy',.01),('izz',.01),('ixy',0),('ixz',0),('iyz',0)]:E.SubElement(ii,tag).text=str(value)
anchor=E.SubElement(m,'joint',name='anchor',type='fixed');E.SubElement(anchor,'parent').text='world';E.SubElement(anchor,'child').text='base'
j=E.SubElement(m,'joint',name='belt_translation',type='prismatic');E.SubElement(j,'parent').text='base';E.SubElement(j,'child').text='body';axis=E.SubElement(j,'axis');E.SubElement(axis,'xyz').text='1 0 0';limit=E.SubElement(axis,'limit');E.SubElement(limit,'lower').text='-.12';E.SubElement(limit,'upper').text='.12';E.SubElement(limit,'effort').text='100';E.SubElement(limit,'velocity').text='.1'
p=E.SubElement(m,'plugin',filename='gz-sim-joint-controller-system',name='gz::sim::systems::JointController');E.SubElement(p,'joint_name').text='belt_translation';E.SubElement(p,'topic').text='/model/conveyor/link/body/track_cmd_vel';E.SubElement(p,'use_force_commands').text='true';E.SubElement(p,'p_gain').text='200';E.SubElement(p,'i_gain').text='0';E.SubElement(p,'d_gain').text='1';E.SubElement(p,'cmd_max').text='100';E.SubElement(p,'cmd_min').text='-100'
p=E.SubElement(m,'plugin',filename='gz-sim-odometry-publisher-system',name='gz::sim::systems::OdometryPublisher');E.SubElement(p,'odom_topic').text='/model/conveyor/link/body/odometry';E.SubElement(p,'odom_publish_frequency').text='50';E.SubElement(p,'dimensions').text='3'
w.find("model[@name='leaf_00']/pose").text='.46 .25 .006 0 0 0';r.write(dst/'world.sdf',encoding='unicode')
m=json.loads((src/'scene_manifest.json').read_text());(dst/'scene_manifest.json').write_text(json.dumps({'leaf_00':m['leaf_00']}));(dst/'evidence').mkdir(exist_ok=True)
(dst/'bench.py').write_text('''import rclpy,json,numpy as np
from runtime import Robot,P
from motion_compensation import predict_position
from feeding import release_into_box
from perception import segment
def centroid(r):
 mask=segment(r.observation_rgb)[0];q=r.cloud
 valid=(mask>0)&np.isfinite(q).all(axis=-1)&(q[:,:,0]>.30)&(q[:,:,0]<.70)&(q[:,:,1]>.05)&(q[:,:,1]<.45)&(q[:,:,2]>.0003)&(q[:,:,2]<.08)
 if valid.sum()<100:raise RuntimeError('Insufficient leaf cloud')
 return np.median(q[valid],axis=0)
rclpy.init();r=Robot();result={'mode':'indexed translating belt top segment, contact transport; vision + encoder prediction; stopped during pinch'}
try:
 r.wait_ready();r.say('TARE');r.belt(0);r.hand(False);r.move([.38,0,.55]);r.hand(True,1.2);zero,_=r.sample();axis=zero/np.linalg.norm(zero);r.hand(False);scale0,_=r.scale_sample()
 r.say('MOVING DETECT');r.move([.28,.25,.68]);r.belt(.02);r.sleep(.7);candidates=r.sense()
 center=centroid(r);mask=segment(r.observation_rgb)[0];q=r.cloud;valid=(mask>0)&np.isfinite(q).all(axis=-1)&(np.linalg.norm(q[:,:,:2]-center[:2],axis=-1)<.15)&(q[:,:,2]>.0003)&(q[:,:,2]<.08)
 _,_,vt=np.linalg.svd(q[valid,:2]-center[:2],full_matrices=False);axis_xy=vt[0];yaw=float((np.arctan2(axis_xy[1],axis_xy[0])+np.pi/2)%np.pi-np.pi/2)
 c=dict(grasp=center.tolist(),yaw=yaw,image_sim_s=r.cloud_sim_s,grasp_source='single visible leaf foreground cloud centroid and PCA axis');r.snapshot('moving-detect')
 if r.encoder is None:raise RuntimeError('Encoder absent')
 velocity=r.encoder.twist.twist.linear.x
 if abs(velocity-.02)>.003:raise RuntimeError(f'Conveyor did not reach commanded speed: {velocity}')
 r.sleep(.5);stop_time=r.simtime;r.belt(0)
 # Infer stop position from the actual image timestamp and encoder; no model poses.
 predicted=predict_position(c['grasp'],[velocity,0,0],stop_time-c['image_sim_s'],0);r.sleep(.6)
 result.update(detection=c,encoder_m_s=velocity,stop_sim_s=stop_time,predicted_xyz=predicted.tolist())
 validation=r.sense();r.snapshot('stopped-check')
 stopped_center=centroid(r)
 result['stopped_visual_error_mm']=float(np.linalg.norm(stopped_center[:2]-predicted[:2])*1000)
 if result['stopped_visual_error_mm']>8:raise RuntimeError('Compensated target disagrees with stopped observation')
 r.say('COMPENSATED GRASP');g=predicted;yaw=c['yaw'];r.move([g[0],g[1],.45],yaw=yaw);r.move([g[0],g[1],max(.001,g[2]-.008)+.1084],duration=5,yaw=yaw,linear=True);r.hand(True,4.,stiffness=500.,closed_position=0.);r.snapshot('closed')
 r.move([g[0],g[1],.45],duration=6,yaw=yaw,linear=True);r.move([.38,0,.55],duration=4);w,_=r.sample();result['wrist_g']=float((w-zero)@axis*1000/9.8);r.snapshot('held')
 if result['wrist_g']<3:raise RuntimeError('No actual grasp')
 r.say('RELEASE TO SCALE');b=r.box_xyz();drop=[float(b[0]),float(b[1]+.075),float(b[2]+.34)];r.move([drop[0],drop[1],.55],duration=4);r.move(drop);release_into_box(r,drop);r.move([.30,-.02,.58]);scale,_=r.scale_sample();result['scale_gain_g']=(scale-scale0)*1000/9.8;r.snapshot('verified')
 result['pass']=result['wrist_g']>8 and abs(result['scale_gain_g']-result['wrist_g'])<1
except Exception as e:result['error']=str(e);result['pass']=False
finally:
 r.belt(0)
 try:r.snapshot('diagnostic-final')
 except Exception:pass
 (P/'evidence/result.json').write_text(json.dumps(result,indent=2));print(result,flush=True);rclpy.shutdown();r.thread.join(3)
''')
print(dst)
