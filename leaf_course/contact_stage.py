import json,traceback,rclpy,numpy as np
from pathlib import Path
import time
from scipy.spatial.transform import Rotation
from runtime import Robot,P
from feeding import release_into_box
from contact_geometry import support_audit
rclpy.init();r=Robot();cfg=json.loads((P/'fixture.json').read_text());result=dict(fixture=cfg,pass_test=False,control='predefined fixture coordinates; no perception/model-pose grasp selection')
mesh_vertices={}
for name,spec in r.manifest.items():
 mesh_vertices[name]=[]
 for part in spec['parts']:
  uri=part['mesh'];path=P/uri
  if not path.exists():path=Path('/tmp/leaf_geometry_v2')/uri
  mesh_vertices[name].append(np.array([list(map(float,s.split()[1:])) for s in path.read_text().splitlines() if s.startswith('v ')]))
def extents(truth):
 return {name:np.concatenate([Rotation.from_quat(st['quat']).apply(v)+st['xyz'] for st,v in zip(states,mesh_vertices[name])]) for name,states in truth.items()}
trace=[]
def record():
 if not r.state or r.actual_hand is None:return
 trace.append(dict(contact_depths={side:[float(depth) for c in msg.contacts for depth in c.depths] for side,(_,msg) in r.contacts.items()},t=r.simtime,hand_xyz=r.hand_pose()[:3,3].tolist(),fingers=[r.state.get('panda_finger_joint1'),r.state.get('panda_finger_joint2')],wrist_q=r.wrist_q,leaf_z={n:[st['xyz'][2] for st in states] for n,states in r.truth().items()}))
try:
 r.wait_ready();timer=r.create_timer(.1,record)
 r.wait_ready();r.hand(False,force=5.);r.sleep(5);first=r.truth();t=r.simtime;r.sleep(1);second=r.truth();dt=r.simtime-t
 support=support_audit({n:[Rotation.from_quat(st['quat']).apply(v)+st['xyz'] for st,v in zip(states,mesh_vertices[n])] for n,states in second.items()});result['support']=support
 if support['unsupported']:raise RuntimeError('Unsupported leaves at rest')
 shape=extents(second);zmin={n:float(q[:,2].min()) for n,q in shape.items()};speed=max(np.max(np.linalg.norm(np.array([s['xyz'] for s in second[n]])-np.array([s['xyz'] for s in first[n]]),axis=1))/dt for n in second)
 result['rest']=dict(min_z_m=zmin,max_patch_speed_m_s=float(speed));r.snapshot('rest')
 if min(zmin.values())<-.001 or min(zmin.values())>.0015 or speed>.003:raise RuntimeError('Resting geometry/settling gate failed')
 r.say('TARE');r.move([.38,0,.55]);r.hand(True,2.5,closed_position=0.);zero,_=r.sample();axis=zero/np.linalg.norm(zero);r.hand(False,force=5.);scale0,_=r.scale_sample()
 yaw=cfg['yaw'];z=cfg['tip_height_m']+.1084;r.say('FIXED GRASP');r.move([.5,.25,.45],yaw=yaw);r.move([.5,.25,z],duration=5,yaw=yaw,linear=True);r.hand(True,2.5,stiffness=250.,closed_position=0.,close_duration=6.);r.snapshot('closed')
 result['ground_contacts']=[c for cs in r.contact_names().values() for c in cs if 'ground_' in c]
 if result['ground_contacts']:raise RuntimeError('Finger contacted ground')
 result['gather']=[]
 for height in [.003,.006,.009,.012]:
  r.move([.5,.25,z+height],duration=1,yaw=yaw,linear=True);r.hand(True,2.5,stiffness=350.,closed_position=0.);r.sleep(1)
  gap=r.state['panda_finger_joint1']+r.state['panda_finger_joint2']-.016;contacts=r.contact_names()
  result['gather'].append(dict(height_m=height,pad_gap_m=gap,contacts=contacts))
  if .0008<gap<.055 and all(any('leaf_' in c for c in contacts.get(side,[])) for side in ['left','right']):break
 if not (.0008<gap<.100 and all(any('leaf_' in c for c in contacts.get(side,[])) for side in ['left','right'])):raise RuntimeError('No bilateral leaf contact after bounded prelift')
 r.hand(True,2.5,stiffness=350.,closed_position=0.);r.snapshot('secured')
 r.move([.5,.25,.45],duration=5,yaw=yaw,linear=True);r.move([.38,0,.55],duration=4);r.sleep(2);load,std=r.sample();mass=float((load-zero)@axis*1000/9.8);expected=sum(x['mass_kg'] for x in r.manifest.values())*1000;held=extents(r.truth());r.snapshot('held')
 contacts=r.contact_names();fresh=all(side in r.contacts and __import__('time').monotonic()-r.contacts[side][0]<2 for side in ['left','right']);pads=all(any(('_pad_collision' in c or 'pinch_bevel' in c) and 'leaf_' in c for c in contacts.get(side,[])) for side in ['left','right']);result.update(held_contacts=contacts,contact_feedback_fresh=fresh,bilateral_pad_contact=pads)
 normals={side:[dict(normals=[[n.x,n.y,n.z] for n in c.normals],depths=list(c.depths)) for c in r.contacts[side][1].contacts if 'leaf_' in c.collision1.name+' '+c.collision2.name] for side in ['left','right'] if side in r.contacts};result['held_contact_details']=normals
 closing_axis=r.hand_pose()[:3,1];opposed=all(any(abs(np.dot(n,closing_axis))>.7 for contact in normals.get(side,[]) for n in contact['normals']) for side in ['left','right']);result['opposing_face_contact']=opposed
 result.update(wrist_g=mass,expected_g=expected,held_min_z_m={n:float(v[:,2].min()) for n,v in held.items()})
 if not fresh or not pads or not opposed:raise RuntimeError('No fresh bilateral pad contact; toe-only support is not a clamp')
 max_depth=max([depth for cs in normals.values() for contact in cs for depth in contact['depths']]+[0.]);result['held_max_penetration_m']=max_depth
 if max_depth>.0005:raise RuntimeError('Held contact penetrates more than 0.5 mm')
 if abs(mass-expected)>1 or any(v[:,2].min()<.15 for v in held.values()):raise RuntimeError('Not all fixture leaves retained after transport/hold')
 r.say('RELEASE');b=r.box_xyz();drop=[float(b[0]),float(b[1]+.075),float(b[2]+.34)];r.move([drop[0],drop[1],.55],duration=4);r.move(drop);release_into_box(r,drop);r.move([.38,0,.55]);scale,_=r.scale_sample();empty,_=r.sample();r.sleep(1);r.snapshot('released');final=extents(r.truth());b=r.box_xyz()
 centers={n:(v.mean(0)-b).tolist() for n,v in final.items()};gain=(scale-scale0)*1000/9.8;residual=float((empty-zero)@axis*1000/9.8)
 result.update(scale_gain_g=gain,residual_wrist_g=residual,box_relative_centers=centers,contacts_after=r.contact_names())
 result['pass_test']=bool(abs(gain-expected)<1 and abs(residual)<1 and all(abs(c[0])<.14 and abs(c[1])<.13 and 0<c[2]<.10 for c in centers.values()) and not any('leaf_' in c for cs in r.contact_names().values() for c in cs))
except Exception as e:result['error']=str(e);traceback.print_exc()
finally:
 (P/'evidence/motion-trace.json').write_text(json.dumps(trace));(P/'evidence/result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);rclpy.shutdown();r.thread.join(3)
