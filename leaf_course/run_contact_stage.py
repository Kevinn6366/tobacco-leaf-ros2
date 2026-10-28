"""Run independent fixed-fixture trials; stop at first failed mechanics gate."""
import argparse,json,subprocess,time,shutil,hashlib
from pathlib import Path
P=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--count',type=int,choices=[1,3],required=True);p.add_argument('--seeds',type=int,nargs='+',default=[1,2,3]);p.add_argument('--variant',default='small-margin');p.add_argument('--test-script',type=Path);p.add_argument('--single-baseline');p.add_argument('--coupled-fingers',action='store_true');p.add_argument('--geometry-root',type=Path,default=Path('/tmp/leaf_geometry_v2'));a=p.parse_args()
archive=P/'evidence'/'geometry-stage'/a.variant;archive.mkdir(parents=True,exist_ok=True)
if a.count==3:
 single_archive=P/'evidence/geometry-stage'/a.single_baseline if a.single_baseline else archive
 if a.single_baseline:
  base=single_archive/'1-1/source'
  for name in ['build_scene.py','leaf_geometry.py','finger_geometry.py','runtime.py','physics/patch_contact.py']:
   current=P/name if name.startswith('physics/') else a.geometry_root/name
   if current.read_bytes()!=(base/Path(name).name).read_bytes():raise SystemExit('Physics/control differs from the specified single-leaf baseline: '+name)
  lib=P/'physics/contact-build/libleaf-contact-physics.so'
  if hashlib.sha256(lib.read_bytes()).hexdigest()!=(base/'plugin-sha256.txt').read_text().strip():raise SystemExit('Physics plugin differs from single-leaf baseline')
 for seed in [1,2,3]:
  f=single_archive/f'1-{seed}'/'result.json'
  if not f.exists() or not json.loads(f.read_text()).get('pass_test'):raise SystemExit('Three independent single-leaf trials must pass before multi-leaf tests')
  if not a.single_baseline:
   source=f.parent/'source'
   config=json.loads((source/'fixture.json').read_text())
   if bool(config.get('mechanical_finger_coupling'))!=a.coupled_fingers:raise SystemExit('Finger coupling differs from single-leaf trials')
   if a.test_script and a.test_script.read_bytes()!=(source/'contact_stage.py').read_bytes():raise SystemExit('Test controller/motion differs from single-leaf trials')
for seed in a.seeds:
 dest=archive/f'{a.count}-{seed}'
 if dest.exists():raise SystemExit(f'Will not overwrite archived trial {dest}')
 subprocess.run(['python3',str(P/'prepare_contact_stage.py'),'--count',str(a.count),'--seed',str(seed),'--geometry-root',str(a.geometry_root)],check=True)
 d=Path(f'/tmp/leaf_contact_v2_{a.count}_{seed}')
 subprocess.run(['bash',str(d/'stop.sh')],check=True)
 if a.test_script:shutil.copy2(a.test_script.resolve(),d/'contact_stage.py')
 if a.coupled_fingers:
  import xml.etree.ElementTree as E
  robot=E.parse(d/'panda.sdf');axis=robot.find(".//joint[@name='panda_finger_joint2']/axis")
  if axis.find('mimic') is not None:raise RuntimeError('Unexpected existing mimic constraint')
  mimic=E.SubElement(axis,'mimic',joint='panda_finger_joint1',axis='axis')
  for key,value in [('multiplier','1'),('offset','0'),('reference','0')]:E.SubElement(mimic,key).text=value
  robot.write(d/'panda.sdf',encoding='unicode')
  fixture=json.loads((d/'fixture.json').read_text());fixture['mechanical_finger_coupling']='q2=q1; native Bullet gear constraint';(d/'fixture.json').write_text(json.dumps(fixture))
 shutil.rmtree(d/'evidence');(d/'evidence').mkdir()
 result=d/'evidence/result.json'
 if result.exists():result.unlink()
 log=d/'contact-small-margin.log'
 if log.exists():log.unlink()
 sim_log=d/'sim.log'
 if sim_log.exists():sim_log.unlink()
 subprocess.run(['python3',str(d/'desktop_exec.py'),'sim'],check=True)
 subprocess.run(['python3',str(d/'desktop_exec.py'),'python3 contact_stage.py',log.name],check=True)
 deadline=time.monotonic()+900
 while not result.exists() and time.monotonic()<deadline:time.sleep(2)
 subprocess.run(['bash',str(d/'stop.sh')],check=True)
 if not result.exists():raise SystemExit('Contact trial timed out; simulation stopped')
 dest=archive/f'{a.count}-{seed}'
 if dest.exists():raise SystemExit(f'Will not overwrite archived trial {dest}')
 shutil.copytree(d/'evidence',dest);shutil.copy2(log,dest/log.name)
 if sim_log.exists():shutil.copy2(sim_log,dest/'sim.log')
 source=dest/'source';source.mkdir()
 for name in ['world.sdf','panda.urdf','panda.sdf','fixture.json','scene_manifest.json','contact_stage.py','contact_geometry.py','runtime.py','feeding.py','build_scene.py','leaf_geometry.py','finger_geometry.py','verify_leaf_geometry.py','course.launch.py','controllers.yaml','camera.json']:
  shutil.copy2(d/name,source/name)
 for name in ['patch_contact.py','build_contact.sh','CMakeLists.txt']:
  shutil.copy2(P/'physics'/name,source/name)
 for mesh in a.geometry_root.glob('*.obj'):shutil.copy2(mesh,source/mesh.name)
 lib=P/'physics/contact-build/libleaf-contact-physics.so'
 (source/'plugin-sha256.txt').write_text(hashlib.sha256(lib.read_bytes()).hexdigest()+'\n')
 outcome=json.loads(result.read_text());print(json.dumps(outcome),flush=True)
 if not outcome.get('pass_test'):raise SystemExit('Mechanics gate failed; later trials blocked')
