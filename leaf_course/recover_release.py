"""Diagnostic physical release of a leaf remaining on an open finger after a stopped run.
Does not mark the failed batch as complete or overwrite its result.
"""
import json,rclpy,numpy as np
from runtime import Robot,P
from feeding import release_into_box
rclpy.init();r=Robot();result={}
try:
 r.wait_ready();r.hand(False);r.move([.38,0,.55]);before,_=r.scale_sample();w,_=r.sample();old=json.loads((P/'evidence/batch-result.json').read_text());zero=np.array(old['wrist_tare_N']);axis=zero/np.linalg.norm(zero);mass=float((w-zero)@axis*1000/9.8);result['held_g']=mass;r.snapshot('release-recovery-before')
 if mass<3:raise RuntimeError('Leaf no longer retained; cannot validate release')
 b=r.box_xyz();drop=[float(b[0]),float(b[1]+.075),float(b[2]+.34)];r.move([drop[0],drop[1],.55],duration=4);r.move(drop);release_into_box(r,drop);r.snapshot('release-recovery-open');r.move([.30,-.02,.58]);after,_=r.scale_sample();r.snapshot('release-recovery-after');result['scale_gain_g']=(after-before)*1000/9.8;result['pass']=abs(result['scale_gain_g']-mass)<1
except Exception as e:result.update(error=str(e),pass_test=False)
finally:
 (P/'evidence/release-recovery.json').write_text(json.dumps(result,indent=2));print(result,flush=True);rclpy.shutdown();r.thread.join(3)
