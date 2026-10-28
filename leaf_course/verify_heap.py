"""Acceptance for heap workflow. Historical conveyor successes cannot pass this gate."""
from pathlib import Path
import json,xml.etree.ElementTree as E
import numpy as np
P=Path(__file__).resolve().parent
m=json.loads((P/'scene_manifest.json').read_text());world=E.parse(P/'world.sdf')
assert len(m)>=24
poses=np.array([v['initial_pose'][:3] for v in m.values()])
assert np.max(poses[:,2])<.06 and np.ptp(poses[:,1])>.15,'Leaves must fall directly onto the ground'
assert np.min(np.diff(np.sort(poses[:,2])))>.0015,'Initial leaf layers must not interpenetrate'
assert world.find('.//model[@name="pile_table"]') is None
assert world.find('.//model[@name="delivery"]') is None
assert world.find('.//model[@name="conveyor"]') is None
box=world.find('.//model[@name="box1"]');assert box is not None
assert box.find('joint') is None,'Box is fixed to world'
assert all(world.find(f'.//model[@name="{name}_region"]') is not None for name in ['source','weigh','delivery'])
r=E.parse(P/'panda.sdf');assert not any('Detachable' in x.get('name','') for x in r.findall('.//plugin'))
b=json.loads((P/'evidence'/'batch-result.json').read_text())
assert b.get('weight_verified') and b.get('box_delivered') and b['complete'],b.get('error','Incomplete')
assert abs(b['delivered_g']-b['target_g'])<=b['tolerance_g']
assert b['box_delivery']['lift'][2]-b['box_delivery']['before'][2]>.06
assert np.linalg.norm(np.array(b['box_delivery']['final'][:2])-[.10,-.53])<.045
assert any(c.get('wrist_mass_g',0)>3 and c.get('plan') for c in b['cycles'])
assert all('ground_' not in str(c.get('contacts',{})) for c in b['cycles'])
print('PASS: heap, adaptive bite plan, wrist feedback, independent scale verification, physical whole-box delivery.')
# Offline physics audit only. These model masses never enter the runtime controller.
max_single_g=max(v['mass_kg']*1000 for v in m.values())
assert any(c.get('outcome')=='deposited' and len(c.get('evaluation_lifted_leaves',[]))>=2 and c.get('wrist_mass_g',0)>max_single_g+3 for c in b['cycles']),'No stable multi-leaf grasp followed by verified deposition'
for c in b['cycles']:
 if c.get('outcome')=='deposited':assert abs(c['wrist_mass_g']-c['change_g'])<3.,'Material lost or wrist/box disagreement'
snapshot=json.loads((P/'evidence/box-delivered-physics.json').read_text());boxpos=np.array(snapshot['box_xyz']);arrived=[]
for name,parts in snapshot['parts'].items():
 center=np.mean([p['xyz'] for p in parts],axis=0)-boxpos
 if abs(center[0])<.14 and abs(center[1])<.13 and -.002<center[2]<.10:arrived.append(name)
arrived_g=sum(m[name]['mass_kg']*1000 for name in arrived)
assert abs(arrived_g-b['delivered_g'])<3.,('Delivered contents differ from verified scale',arrived_g,b['delivered_g'])
assert abs(arrived_g-b['target_g'])<=b['tolerance_g'],('Actual delivered leaf mass outside batch tolerance',arrived_g,b['target_g'])
print(f'PASS: offline content audit {len(arrived)} leaves, {arrived_g:.3f} g at delivery station.')
