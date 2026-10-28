"""Structural geometry checks; physics contact margins still require drop tests."""
from pathlib import Path
import json,xml.etree.ElementTree as E
from collections import Counter
import numpy as np
from scipy.spatial import ConvexHull
P=Path(__file__).resolve().parent
world=E.parse(P/'world.sdf');manifest=json.loads((P/'scene_manifest.json').read_text());count=0
for name,spec in manifest.items():
 assert abs(sum(p['mass_kg'] for p in spec['parts'])-spec['mass_kg'])<1e-10
 for part in spec['parts']:
  link=world.find(f".//model[@name='{name}']/link[@name='{part['link']}']")
  a=link.find("visual[@name='lamina']/geometry/mesh/uri").text;b=link.find('collision/geometry/mesh/uri').text
  assert a==b and link.find('collision/pose') is None and link.find("visual[@name='lamina']/pose") is None
  vs=[];fs=[]
  for line in (P/part['mesh']).read_text().splitlines():
   z=line.split()
   if z[0]=='v':vs.append(list(map(float,z[1:])))
   elif z[0]=='f':fs.append([int(i)-1 for i in z[1:]])
  v=np.array(vs);f=np.array(fs);edges=Counter(tuple(sorted((x,y))) for t in f for x,y in zip(t,np.roll(t,-1)))
  assert all(n==2 for n in edges.values()),'Open shell'
  volume=sum(np.dot(v[a],np.cross(v[b],v[c])) for a,b,c in f)/6
  assert volume>0 and abs(volume-ConvexHull(v).volume)<1e-12,'Not an exact convex solid'
  I=link.find('inertial/inertia');val=lambda k:float(I.find(k).text)
  tensor=np.array([[val('ixx'),val('ixy'),val('ixz')],[val('ixy'),val('iyy'),val('iyz')],[val('ixz'),val('iyz'),val('izz')]])
  assert np.linalg.eigvalsh(tensor).min()>0
  assert len(link.find('collision').findall('surface'))==1
  assert link.find('collision/surface/friction/bullet/friction') is not None
  count+=1
robot=E.parse(P/'panda.urdf')
for side in ['left','right']:
 link=robot.find(f"link[@name='panda_{side}finger']")
 assert len(link.findall('visual'))==len(link.findall('collision'))==2
 v=link.find("visual[geometry/box]") if False else next(v for v in link.findall('visual') if v.find('geometry/box') is not None)
 c=next(c for c in link.findall('collision') if c.find('geometry/box') is not None)
 assert v.find('geometry/box').get('size')==c.find('geometry/box').get('size')
 assert v.find('origin').get('xyz')==c.find('origin').get('xyz')
 vm=next(v.find('geometry/mesh') for v in link.findall('visual') if v.find('geometry/mesh') is not None)
 cm=next(c.find('geometry/mesh') for c in link.findall('collision') if c.find('geometry/mesh') is not None)
 assert vm.get('filename')==cm.get('filename')
 vv=[];ff=[]
 for line in Path(vm.get('filename')).read_text().splitlines():
  z=line.split()
  if z[0]=='v':vv.append(list(map(float,z[1:])))
  elif z[0]=='f':ff.append([int(i)-1 for i in z[1:]])
 vv=np.array(vv);ff=np.array(ff);edge=Counter(tuple(sorted((a,b))) for face in ff for a,b in zip(face,np.roll(face,-1)))
 assert all(n==2 for n in edge.values())
 volume=sum(np.dot(vv[a],np.cross(vv[b],vv[c])) for a,b,c in ff)/6
 assert volume>0 and abs(volume-ConvexHull(vv).volume)<1e-12
 box_volume=float(np.prod(np.fromstring(c.find('geometry/box').get('size'),sep=' ')))
 centroid=(box_volume*np.fromstring(c.find('origin').get('xyz'),sep=' ')+volume*vv.mean(0))/(box_volume+volume)
 assert np.allclose(centroid,np.fromstring(link.find('inertial/origin').get('xyz'),sep=' '),atol=1e-9)
for co in E.parse(P/'panda.sdf').findall(".//link"):
 if 'finger' in co.get('name',''):
  for collision in co.findall('collision'):
   assert len(collision.findall('surface'))==1
   assert collision.find('surface/friction/bullet/friction') is not None
limits=[float(robot.find(f"joint[@name='panda_finger_joint{i}']/limit").get('lower')) for i in [1,2]]
assert sum(limits)-.016>=0,'Opposing pads can overlap at joint limits'
result=dict(pass_test=True,leaves=len(manifest),convex_closed_parts=count,shared_visual_collision_geometry=True,structural_shape_difference=0,checks=['identical mesh URI and transform','closed oriented shell','mesh volume equals convex hull','positive inertia','mass conservation','matched finger pad visual/collision/center of mass','nonoverlapping finger joint limits','single explicit Bullet contact material'],limitation='Does not certify physics-engine collision margins or grasp stability')
(P/'geometry-check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
