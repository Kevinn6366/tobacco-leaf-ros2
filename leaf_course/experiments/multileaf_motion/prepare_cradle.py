from pathlib import Path
import shutil,subprocess,argparse
parser=argparse.ArgumentParser();parser.add_argument('--source',type=Path,default=Path('/tmp/leaf_geometry_v2'));parser.add_argument('--output',type=Path,default=Path('/tmp/leaf_geometry_cradle'));args=parser.parse_args()
src=args.source;dst=args.output
if dst.exists():raise SystemExit('Refusing to overwrite candidate geometry')
shutil.copytree(src,dst)
# Twelve-millimetre inward lead-in supports a folded 26 mm wing while the
# original vertical pad supplies the opposing clamp contact. Same closed mesh
# is rendered and collided; mass properties are recomputed by finger().
p=dst/'finger_geometry.py';s=p.read_text();assert s.count('[-.008,.050]')==1;s=s.replace('[-.008,.050]','[-.017,.050]');p.write_text(s)
p=dst/'build_scene.py';s=p.read_text().replace("set('lower','.0081')","set('lower','.0171')");p.write_text(s)
p=dst/'verify_leaf_geometry.py';s=p.read_text().replace('sum(limits)-.016>=0','sum(limits)-.034>=0');p.write_text(s)
subprocess.run(['python3','build_scene.py'],cwd=dst,check=True)
subprocess.run(['python3','verify_leaf_geometry.py'],cwd=dst,check=True)
