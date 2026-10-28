"""Fixed fixture test only. No vision or batch planner supplies grasp coordinates."""
from pathlib import Path
import argparse,shutil,json,xml.etree.ElementTree as E
import numpy as np
p=argparse.ArgumentParser();p.add_argument('--count',type=int,choices=[1,3],default=1);p.add_argument('--seed',type=int,default=1);p.add_argument('--geometry-root',type=Path,default=Path('/tmp/leaf_geometry_v2'));p.add_argument('--output',type=Path);a=p.parse_args()
src=a.geometry_root;dst=a.output or Path(f'/tmp/leaf_contact_v2_{a.count}_{a.seed}');dst.mkdir(exist_ok=True)
for f in src.iterdir():
 if f.is_file() and f.suffix in ['.py','.yaml','.urdf','.sdf','.json','.sh','.obj']:shutil.copy2(f,dst/f.name)
rng=np.random.default_rng(a.seed);yaw=[0.,.08,-.08][(a.seed-1)%3];names=[f'leaf_{i:02d}' for i in range(a.count)]
w=E.parse(src/'world.sdf');world=w.getroot().find('world')
for m in list(world.findall('model')):
 if not(m.get('name').startswith('ground') or m.get('name') in names+['scale','box1']):world.remove(m)
for i,name in enumerate(names):
 xy=np.array([.5,.25])+rng.uniform(-.002,.002,2);angle=yaw+rng.uniform(-.025,.025)
 world.find(f"model[@name='{name}']/pose").text=f'{xy[0]} {xy[1]} {.006+i*.004} 0 0 {angle}'
w.write(dst/'world.sdf',encoding='unicode');manifest=json.loads((src/'scene_manifest.json').read_text());(dst/'scene_manifest.json').write_text(json.dumps({n:manifest[n] for n in names}));(dst/'fixture.json').write_text(json.dumps(dict(count=a.count,seed=a.seed,yaw=yaw,tip_height_m=.0001,force_schedule_N=dict(tare=2.5,initial=2.5,gather=2.5,hold=2.5,open=5.))));(dst/'evidence').mkdir(exist_ok=True)
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("'ROS_DOMAIN_ID']='32'","'ROS_DOMAIN_ID']='41'").replace("'GZ_PARTITION']='leaf_course'","'GZ_PARTITION']='leaf_contact_v2'"))
shutil.copy2(Path(__file__).with_name('contact_stage.py'),dst/'contact_stage.py');print(dst)

plugin_path = Path(__file__).resolve().parent/'physics/contact-build/libleaf-contact-physics.so'
p=dst/'desktop_exec.py';p.write_text(p.read_text().replace("name=sys.argv[1]", f"env['LEAF_PHYSICS_ENGINE']={str(plugin_path)!r}\nname=sys.argv[1]"))
