"""Reproducible 24-leaf contact-only Gazebo course scene. No grasp constraints."""
from pathlib import Path
import xml.etree.ElementTree as E
import math,json,numpy as np,subprocess
from leaf_geometry import patch,properties,prism,write_obj,surface_veins,THICKNESS,STATIONS
from finger_geometry import finger
P=Path(__file__).resolve().parent
rng=np.random.default_rng(20261008)
def sub(p,t,text=None,**a):
 e=E.SubElement(p,t,a)
 if text is not None:e.text=str(text)
 return e
def plugin(p,lib,cls):return sub(p,'plugin',filename='gz-sim-'+lib+'-system',name='gz::sim::systems::'+cls)
def boxgeom(p,size):sub(sub(p,'geometry'),'box').append(E.fromstring('<size>'+size+'</size>'))
def color(v,c):
 m=sub(v,'material');sub(m,'diffuse',c);sub(m,'ambient',c)
def inertia(l,m,size):
 a,b,c=size;i=sub(l,'inertial');sub(i,'mass',m);ii=sub(i,'inertia')
 for k,v in dict(ixx=m*(b*b+c*c)/12,iyy=m*(a*a+c*c)/12,izz=m*(a*a+b*b)/12,ixy=0,ixz=0,iyz=0).items():sub(ii,k,v)
def friction(c,mu):
 surfaces=c.findall('surface');s=surfaces[0] if surfaces else sub(c,'surface')
 for extra in surfaces[1:]:c.remove(extra)
 for old in s.findall('friction'):s.remove(old)
 f=sub(s,'friction');o=sub(f,'ode');sub(o,'mu',mu);sub(o,'mu2',mu)
 b=sub(f,'bullet');sub(b,'friction',mu);sub(b,'friction2',mu);sub(b,'rolling_friction',0.)
 # Bullet's default spinning-friction coefficient is 1 (metre-scale torque arm).
 # Thin rubber/lamina contact has a sub-millimetre effective torsional radius.
 sub(sub(f,'torsional'),'coefficient',.0001)
def visual(l,name,pose,size,c):
 v=sub(l,'visual',name=name);sub(v,'pose',pose);boxgeom(v,size);color(v,c)
r=E.parse(P/'panda-base.urdf').getroot()
j=r.find("joint[child[@link='panda_hand']]") if False else next(j for j in r.findall('joint') if j.find('child').get('link')=='panda_hand')
# Series-elastic axial wrist load cell, independent of finger actuator wrench reporting.
j.set('type','prismatic');sub(j,'axis',xyz='0 0 1');sub(j,'limit',lower='-.003',upper='.003',effort='150',velocity='.2');sub(j,'dynamics',damping='80')
g=sub(r,'gazebo',reference=j.get('name'));sub(g,'preserveFixedJoint','true')
s=sub(g,'sensor',name='wrist_ft',type='force_torque');sub(s,'always_on','true');sub(s,'update_rate',100);sub(s,'topic','/panda/wrist/wrench');ft=sub(s,'force_torque');sub(ft,'frame','child');sub(ft,'measure_direction','child_to_parent')
for side,name in [('left','panda_leftfinger'),('right','panda_rightfinger')]:
 l=r.find(f"link[@name='{name}']")
 # Matched custom finger/pad bodies: avoid the source model's 10:1 mass mismatch.
 l.find('inertial/mass').set('value','.15')
 vertices,faces,center,tensor=finger(side)
 l.find('inertial/origin').set('xyz',' '.join(map(str,center)))
 ii=l.find('inertial/inertia')
 for key,a,b in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:ii.set(key,str(tensor[a,b]))
 for c in l.findall('collision')+l.findall('visual'):l.remove(c)
 c=sub(l,'collision',name=side+'_pad');sub(c,'origin',xyz='0 0 .025');sub(sub(c,'geometry'),'box',size='.028 .010 .050')
 mesh=P/(side+'_pinch_bevel.obj');write_obj(mesh,vertices,faces)
 for tag in ['visual','collision']:
  el=sub(l,tag,name=side+'_pinch_bevel');sub(sub(el,'geometry'),'mesh',filename=str(mesh))
 v=sub(l,'visual');sub(v,'origin',xyz='0 0 .025');sub(sub(v,'geometry'),'box',size='.028 .010 .050');mat=sub(v,'material',name=side+'_rubber');sub(mat,'color',rgba='.09 .12 .13 1')
 g=sub(r,'gazebo',reference=name);sub(g,'mu1',1.4);sub(g,'mu2',1.4)
 s=sub(g,'sensor',name=side+'_contact',type='contact');sub(s,'always_on','true');sub(s,'update_rate',100);sub(s,'topic','/gripper/'+side+'/contacts');sub(sub(s,'contact'),'collision',side+'_pad')
# Custom wide-stroke jaw carriage: rail supports the fingers throughout 65 mm travel.
hand=r.find("link[@name='panda_hand']")
for tag in ['visual','collision']:
 el=sub(hand,tag,name='wide_jaw_rail');sub(el,'origin',xyz='0 0 .055');sub(sub(el,'geometry'),'box',size='.040 .150 .012')
 if tag=='visual':mat=sub(el,'material',name='rail_grey');sub(mat,'color',rgba='.40 .45 .48 1')
for i in (1,2):
 r.find(f"joint[@name='panda_finger_joint{i}']/limit").set('upper','.065')
 r.find(f"joint[@name='panda_finger_joint{i}']/limit").set('lower','.0081')
 r.find(f"joint[@name='panda_finger_joint{i}']/dynamics").set('damping','2.0')
 j=r.find(f"ros2_control/joint[@name='panda_finger_joint{i}']");j.find('command_interface').set('name','effort');sub(j,'state_interface',name='effort')
# Eye-in-hand depth camera, same tilted mounting as prior stereo camera.
l=sub(r,'link',name='wrist_camera');i=sub(l,'inertial');sub(i,'mass',value='.085');sub(i,'inertia',ixx='.00007',iyy='.00004',izz='.00008',ixy='0',ixz='0',iyz='0')
v=sub(l,'visual');sub(sub(v,'geometry'),'box',size='.027 .080 .022');m=sub(v,'material',name='camera_blue');sub(m,'color',rgba='.08 .25 .45 1')
j=sub(r,'joint',name='wrist_camera_joint',type='fixed');sub(j,'parent',link='panda_hand');sub(j,'child',link='wrist_camera');sub(j,'origin',xyz='-.075 0 .035',rpy=f'0 {-math.pi/2+.60} 0')
g=sub(r,'gazebo',reference='wrist_camera');sub(g,'preserveFixedJoint','true');s=sub(g,'sensor',name='wrist_left',type='camera');sub(s,'always_on','true');sub(s,'update_rate',4);sub(s,'topic','/wrist/stereo/left/image');c=sub(s,'camera');sub(c,'horizontal_fov',1.0471975512);im=sub(c,'image');sub(im,'width',640);sub(im,'height',480);sub(im,'format','R8G8B8');cl=sub(c,'clip');sub(cl,'near',.03);sub(cl,'far',3)
# Parallel right RGB camera, 60 mm to the right in optical coordinates.
l=sub(r,'link',name='wrist_camera_right');i=sub(l,'inertial');sub(i,'mass',value='.001');sub(i,'inertia',ixx='.000001',iyy='.000001',izz='.000001',ixy='0',ixz='0',iyz='0')
j=sub(r,'joint',name='wrist_camera_right_joint',type='fixed');sub(j,'parent',link='wrist_camera');sub(j,'child',link='wrist_camera_right');sub(j,'origin',xyz='0 -.060 0',rpy='0 0 0')
g=sub(r,'gazebo',reference='wrist_camera_right');sub(g,'preserveFixedJoint','true')
s=sub(g,'sensor',name='wrist_right',type='camera');sub(s,'always_on','true');sub(s,'update_rate',4);sub(s,'topic','/wrist/stereo/right/image');c=sub(s,'camera');sub(c,'horizontal_fov',1.0471975512);im=sub(c,'image');sub(im,'width',640);sub(im,'height',480);sub(im,'format','R8G8B8');cl=sub(c,'clip');sub(cl,'near',.03);sub(cl,'far',3)
r.find("gazebo/plugin[@name='gz_ros2_control::GazeboSimROS2ControlPlugin']/parameters").text=str(P/'controllers.yaml')
E.indent(r);E.ElementTree(r).write(P/'panda.urdf',encoding='unicode')
sdf=E.Element('sdf',version='1.9');w=sub(sdf,'world',name='leaf_course');ph=sub(w,'physics',name='0.5ms',type='ignored');sub(ph,'max_step_size',.0005);sub(ph,'real_time_factor',1);sub(ph,'max_contacts',2);dart=sub(ph,'dart');sub(dart,'collision_detector','bullet');sub(sub(dart,'solver'),'solver_type','pgs')
for lib,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('forcetorque','ForceTorque'),('contact','Contact'),('sensors','Sensors')]:
 pl=plugin(w,lib,cls)
 if cls=='Sensors':sub(pl,'render_engine','ogre')
scene=sub(w,'scene');sub(scene,'ambient','.7 .7 .7 1');sub(scene,'background','.09 .12 .16 1')
l=sub(w,'light',name='sun',type='directional');sub(l,'pose','0 0 3 0 0 0');sub(l,'diffuse','.8 .8 .8 1');sub(l,'direction','.1 .2 -1')
def box(name,pose,size,c):
 m=sub(w,'model',name=name);sub(m,'static','true');sub(m,'pose',pose);l=sub(m,'link',name='body');visual(l,'visual','0 0 0 0 0 0',size,c);co=sub(l,'collision',name='collision');boxgeom(co,size);friction(co,.8);return m
# Continuous ground with a flush weighing patch; three markings, no raised tables.
# Split only the collision slab around the scale so the ground cannot bypass its spring.
box('ground_west','-.84 0 -.025 0 0 0','2.32 5 .05','.14 .18 .22 1')
box('ground_east','1.33 0 -.025 0 0 0','1.34 5 .05','.14 .18 .22 1')
box('ground_north','.49 1.19 -.025 0 0 0','.34 2.62 .05','.14 .18 .22 1')
box('ground_south','.49 -1.47 -.025 0 0 0','.34 2.06 .05','.14 .18 .22 1')
def region(name,x,y,sx,sy,c):
 m=sub(w,'model',name=name);sub(m,'static','true');l=sub(m,'link',name='marking')
 for k,(dx,dy,ax,ay) in enumerate([(-sx/2,0,.004,sy),(sx/2,0,.004,sy),(0,-sy/2,sx,.004),(0,sy/2,sx,.004)]):
  visual(l,'line'+str(k),f'{x+dx} {y+dy} .00015 0 0 0',f'{ax} {ay} .0002',c)
region('source_region',.50,.25,.48,.40,'.80 .67 .22 1')
region('weigh_region',.49,-.28,.35,.33,'.25 .65 .92 1')
region('delivery_region',.10,-.53,.34,.32,'.20 .75 .40 1')
# The 0.25 kg tray plus the empty 0.16 kg box deflect 2.679 mm: empty surface is z=0.
scale=box('scale','.49 -.28 -.0273213333 0 0 0','.34 .32 .06','.16 .23 .29 1')
scale.find('static').text='false';tl=scale.find('link');tl.set('name','tray');inertia(tl,.25,(.34,.32,.06));tl.find('collision').set('name','tray_contact')
base=sub(scale,'link',name='base');inertia(base,1,(.04,.04,.04));j=sub(scale,'joint',name='anchor',type='fixed');sub(j,'parent','world');sub(j,'child','base')
j=sub(scale,'joint',name='loadcell',type='prismatic');sub(j,'parent','base');sub(j,'child','tray');a=sub(j,'axis');sub(a,'xyz','0 0 1');lim=sub(a,'limit');sub(lim,'lower',-.015);sub(lim,'upper',.002);sub(lim,'effort',100);sub(lim,'velocity',1);d=sub(a,'dynamics');sub(d,'spring_stiffness',1500);sub(d,'spring_reference',0);sub(d,'damping',12)
pp=plugin(scale,'joint-state-publisher','JointStatePublisher');sub(pp,'joint_name','loadcell');sub(pp,'topic','/scale/joints')
sensor=sub(tl,'sensor',name='scale_contacts',type='contact');sub(sensor,'always_on','true');sub(sensor,'update_rate',100);sub(sensor,'topic','/scale/contacts');sub(sub(sensor,'contact'),'collision','tray_contact')
# Box has no world joint: fingers must physically lift it off the load platform.
tray=sub(w,'model',name='box1');sub(tray,'pose','.49 -.28 .010 0 0 0');tl=sub(tray,'link',name='body');inertia(tl,.16,(.30,.28,.09))
boxparts=[('floor',[0,0,0],[.30,.28,.016]),('back',[0,-.145,.043],[.32,.01,.10]),('front',[0,.145,.043],[.32,.01,.10]),('left',[-.155,0,.043],[.01,.28,.10]),('right',[.155,0,.043],[.01,.28,.10]),('handle_left',[-.095,0,.095],[.012,.014,.18]),('handle_right',[.095,0,.095],[.012,.014,.18]),('handle',[0,0,.18],[.20,.014,.016])]
for name,pose,size in boxparts:
 pose=' '.join(map(str,pose+[0,0,0]));size=' '.join(map(str,size));visual(tl,name,pose,size,'.10 .30 .50 1');co=sub(tl,'collision',name=name);sub(co,'pose',pose);boxgeom(co,size);friction(co,1.8)
pp=plugin(tray,'pose-publisher','PosePublisher')
for k,v in dict(publish_model_pose='true',publish_link_pose='true',use_pose_vector_msg='true',update_frequency=15).items():sub(pp,k,v)
(P/'box_geometry.json').write_text(json.dumps(boxparts))

manifest={}
for idx in range(24):
 group=0;layer=idx//6;name=f'leaf_{idx:02d}';x=.50+rng.uniform(-.055,.055);y=.25+rng.uniform(-.12,.12);yaw=rng.uniform(-1.1,1.1);z=.003+idx*.0022
 m=sub(w,'model',name=name);sub(m,'pose',f'{x} {y} {z} 0 0 {yaw}');sub(m,'self_collide','true');pp=plugin(m,'pose-publisher','PosePublisher')
 for k,v in dict(publish_model_pose='true',publish_link_pose='true',use_pose_vector_msg='true',update_frequency=15).items():sub(pp,k,v)
 parts=[];mass=float(rng.uniform(.010,.013));c=[.43+rng.uniform(0,.11),.22+rng.uniform(0,.06),.055,1.]
 def hinge(name,parent,child,position,axis,stiffness,rest):
  j=sub(m,'joint',name=name,type='revolute');sub(j,'parent',parent);sub(j,'child',child);sub(j,'pose',' '.join(map(str,position+[0,0,0])),relative_to='__model__')
  a=sub(j,'axis');sub(a,'xyz',axis);lim=sub(a,'limit');sub(lim,'lower',-1.5);sub(lim,'upper',1.5);sub(lim,'effort',.1);sub(lim,'velocity',20)
  d=sub(a,'dynamics');sub(d,'damping',.000025);sub(d,'spring_stiffness',stiffness);sub(d,'spring_reference',rest)
 # Three spine sections, each with two independently bending lamina wings.
 # This is a 2-D discrete elastic sheet approximation, not a continuous FEM material.
 total_area=sum(properties(patch(k,row)[1],1.)[0] for k in range(3) for row in [0,-1,1])
 for k in range(3):
  for row in [0,-1,1]:
   number=k*3+({0:0,-1:1,1:2}[row]);link=f'l{idx}_p{number}'
   origin,polygon=patch(k,row);cx,cy,_=origin;area=properties(polygon,1.)[0];part_mass=mass*area/total_area
   _,center,moments=properties(polygon,part_mass)
   l=sub(m,'link',name=link);sub(l,'pose',' '.join(map(str,origin))+' 0 0 0')
   inertial=sub(l,'inertial');sub(inertial,'pose',' '.join(map(str,center))+' 0 0 0');sub(inertial,'mass',part_mass);ii=sub(inertial,'inertia')
   for key,value in moments.items():sub(ii,key,value)
   mesh=f'{name}_p{number}.obj';vertices,faces=prism(polygon);write_obj(P/mesh,vertices,faces)
   # Both systems consume the identical closed convex solid, with identical pose/scale.
   for tag,label in [('visual','lamina'),('collision','lamina_contact')]:
    element=sub(l,tag,name=label);sub(sub(sub(element,'geometry'),'mesh'),'uri','file://'+str(P/mesh))
    if tag=='visual':color(element,' '.join(map(str,c)))
    else:friction(element,1.8);sub(element,'max_contacts',4)
   vv,ff=surface_veins(polygon,row);vein_mesh=P/f'{name}_p{number}_veins.obj';write_obj(vein_mesh,vv,ff)
   v=sub(l,'visual',name='surface_ink');sub(sub(sub(v,'geometry'),'mesh'),'uri','file://'+str(vein_mesh));color(v,'.28 .14 .03 1')
   if row==0 and k:hinge(f'l{idx}_spine{k}',f'l{idx}_p{(k-1)*3}',link,[float(STATIONS[k,0]),0,0],'0 1 0',.001,(-1)**k*.10)
   if row:
    # The side boundary is slanted on a tapered prism: rotate about that actual edge.
    x0,w0=STATIONS[k];x1,w1=STATIONS[k+1];axis=np.array([x1-x0,row*(w1-w0)/6,0.]);axis/=np.linalg.norm(axis)
    hinge(f'l{idx}_wing{k}_{row}',f'l{idx}_p{k*3}',link,[float(cx),float(row*(w0+w1)/12),0.],' '.join(map(str,axis)),.0012,row*.65)
   bounds=np.ptp(vertices,axis=0)
   parts.append(dict(link=link,mesh=mesh,collision_mesh=mesh,color=c,pose=origin.tolist()+[0,0,0],size=bounds.tolist(),mass_kg=part_mass,inertial_center=center,polygon_xy=polygon.tolist()))
 manifest[name]=dict(initial_pose=[x,y,z,0,0,yaw],mass_kg=mass,group=group,material_model="2D elastic hinged lamina: spine + lateral bending",parts=parts)
E.indent(sdf);E.ElementTree(sdf).write(P/'world.sdf',encoding='unicode');(P/'scene_manifest.json').write_text(json.dumps(manifest,indent=2))
(P/'camera.json').write_text(json.dumps(dict(width=640,height=480,fps=4,fx=640/(2*math.tan(math.pi/6)),fy=640/(2*math.tan(math.pi/6)),cx=319.5,cy=239.5,hand_xyz=[-.075,0,.035],hand_rpy=[0,-math.pi/2+.60,0],belt_z=0.,ground_z=0.,baseline_m=.060,depth_source='stereo RGB disparity',calibration='ideal simulator intrinsics and rigid extrinsics'),indent=2))
print('Built 24 leaves, 216 dynamic patches, 192 two-direction spring hinges. No attachment plugin.')

# Patch converted SDF: URDF conversion renames collisions and drops named-pad friction.
robot_sdf=E.fromstring(subprocess.check_output(['gz','sdf','-p',str(P/'panda.urdf')],text=True))
for side in ['left','right']:
 link=robot_sdf.find(f"model/link[@name='panda_{side}finger']")
 sensor=link.find('sensor');contact=sensor.find('contact')
 for old in list(contact):contact.remove(old)
 for co in link.findall('collision'):
  friction(co,1.8);sub(co,'max_contacts',4);sub(contact,'collision',co.get('name'))

wj=robot_sdf.find("model/joint[@name='panda_hand_joint']");wj.set('type','prismatic');axis=wj.find('axis');axis.find('xyz').text='0 0 1';dyn=axis.find('dynamics')
for key,value in [('spring_stiffness',20000),('spring_reference',0),('damping',80)]:
 el=dyn.find(key)
 if el is None:el=sub(dyn,key)
 el.text=str(value)
pp=plugin(robot_sdf.find('model'),'joint-state-publisher','JointStatePublisher');sub(pp,'joint_name','panda_hand_joint');sub(pp,'topic','/panda/wrist/loadcell')
pp=plugin(robot_sdf.find('model'),'pose-publisher','PosePublisher')
for key,value in dict(publish_model_pose='true',publish_link_pose='true',use_pose_vector_msg='true',update_frequency=30).items():sub(pp,key,value)
E.indent(robot_sdf);E.ElementTree(robot_sdf).write(P/'panda.sdf',encoding='unicode')
