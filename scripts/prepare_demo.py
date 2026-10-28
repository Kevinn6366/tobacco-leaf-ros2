"""Build the verified three-leaf fixture in a relocatable directory."""
from pathlib import Path
import subprocess, shutil, tempfile, xml.etree.ElementTree as ET, json
root=Path(__file__).resolve().parents[1];course=root/'leaf_course';out=course/'live_mechanics'
if out.exists():raise SystemExit('live_mechanics exists; preserve or move it before rebuilding')
def run(*args,cwd=course):subprocess.run(args,cwd=cwd,check=True)
if not (course/'physics/contact-build/libleaf-contact-physics.so').exists():raise SystemExit('Run bash leaf_course/physics/build_contact.sh first')
with tempfile.TemporaryDirectory(prefix='leaf_demo_build_') as temp:
 base=Path(temp)/'base';cradle=Path(temp)/'cradle'
 run('python3','prepare_geometry_stage.py','--output',str(base));run('python3','build_scene.py',cwd=base)
 run('python3','experiments/multileaf_motion/prepare_cradle.py','--source',str(base),'--output',str(cradle))
 fixture=Path(temp)/'fixture'
 run('python3','prepare_contact_stage.py','--count','3','--seed','1','--geometry-root',str(cradle),'--output',str(fixture))
 shutil.copytree(fixture,out)
 # Builders produce absolute resource references; relocate only these paths.
 for f in out.iterdir():
  if f.suffix in {'.py','.urdf','.sdf','.yaml','.json','.sh'}:
   text=f.read_text()
   for old in [str(base),str(cradle),str(fixture)]:text=text.replace(old,str(out))
   f.write_text(text)
shutil.copy2(course/'experiments/multileaf_motion/cradle.py',out/'contact_stage.py')
shutil.copy2(course/'demo.config',out/'demo.config')
robot=ET.parse(out/'panda.sdf');axis=robot.find(".//joint[@name='panda_finger_joint2']/axis")
mimic=ET.SubElement(axis,'mimic',joint='panda_finger_joint1',axis='axis')
for k,v in [('multiplier','1'),('offset','0'),('reference','0')]:ET.SubElement(mimic,k).text=v
robot.write(out/'panda.sdf',encoding='unicode')
config=json.loads((out/'fixture.json').read_text());config['mechanical_finger_coupling']='q2=q1; native Bullet gear constraint';(out/'fixture.json').write_text(json.dumps(config))
print(out)
