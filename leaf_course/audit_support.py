"""Offline support audit of recorded physics poses; never used to drive grasping."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

def audit(manifest, first, second, dt):
 bodies={}
 for name,spec in manifest.items():
  bodies[name]=[]
  for part,state in zip(spec['parts'],second['parts'][name]):
   half=np.array(part['size'])/2*np.array([.96,.92,1.])
   R=Rotation.from_quat(state['quat']).as_matrix();c=np.array(state['xyz']);extent=np.abs(R)@half
   bodies[name].append((c,R,half,c-extent,c+extent))
 def gap(a,b):
  ca,ra,ha,la,ua=a;cb,rb,hb,lb,ub=b
  if np.any(np.maximum(la,lb)-np.minimum(ua,ub)>.002):return np.inf
  axes=[*ra.T,*rb.T]+[np.cross(x,y) for x in ra.T for y in rb.T]
  return max(abs((cb-ca)@ax)-np.sum(ha*np.abs(ra.T@ax))-np.sum(hb*np.abs(rb.T@ax)) for v in axes if np.linalg.norm(v)>1e-8 for ax in [v/np.linalg.norm(v)])
 grounded={n for n,bs in bodies.items() if min(b[3][2] for b in bs)<=.002}
 graph={n:set() for n in bodies};names=list(bodies)
 for i,n in enumerate(names):
  for m in names[i+1:]:
   if any(gap(a,b)<=.002 for a in bodies[n] for b in bodies[m]):graph[n].add(m);graph[m].add(n)
 supported=set(grounded);queue=list(grounded)
 while queue:
  for m in graph[queue.pop()]-supported:supported.add(m);queue.append(m)
 speed={n:float(np.max(np.linalg.norm(np.array([s['xyz'] for s in second['parts'][n]])-np.array([s['xyz'] for s in first['parts'][n]]),axis=1))/dt) for n in names}
 minz=min(b[3][2] for bs in bodies.values() for b in bs)
 return dict(method='OBB separating-axis proximity graph, linked patches grouped by physical leaf joints',contact_tolerance_m=.002,leaf_count=len(names),direct_ground_support=sorted(grounded),supported_by_ground_or_leaf_chain=sorted(supported),unsupported=sorted(set(names)-supported),min_collision_z_m=float(minz),max_patch_speed_m_s=max(speed.values()),patch_speed_by_leaf=speed,settled=not(set(names)-supported) and minz>-.002 and max(speed.values())<.005)
if __name__=='__main__':
 p=Path(__file__).resolve().parent;e=p/'evidence'
 a=json.loads((e/'scene-start-physics.json').read_text());b=json.loads((e/'scene-settled-physics.json').read_text())
 dt=b.get('sim_time_s',1.)-a.get('sim_time_s',0.)
 manifest=json.loads((p/'scene_manifest.json').read_text())
 missing=sorted(set(manifest)-set(a['parts']) | set(manifest)-set(b['parts']))
 r=dict(settled=False,error='Incomplete recorded poses',missing=missing) if missing else audit(manifest,a,b,dt)
 (e/'support-audit.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
