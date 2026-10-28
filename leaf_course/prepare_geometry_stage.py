from pathlib import Path
import shutil,argparse
parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=Path('/tmp/leaf_geometry_v2'));args=parser.parse_args()
src=Path(__file__).resolve().parent;dst=args.output;dst.mkdir(parents=True,exist_ok=True)
files=['build_scene.py','contact_geometry.py','leaf_geometry.py','finger_geometry.py','verify_leaf_geometry.py','panda-base.urdf','runtime.py','feeding.py','perception.py','orientation.py','height_regions.py','stereo.py','kinematics.py','controllers.yaml','course.launch.py','desktop_exec.py','stop.sh']
for name in files:shutil.copy2(src/name,dst/name)
print(dst)
