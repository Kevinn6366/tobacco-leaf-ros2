"""One controller only: run after stopping the panel. Saves raw static sensor statistics."""
import json,time,numpy as np,rclpy
from runtime import Robot,P
rclpy.init();r=Robot();rows=[]
try:
 r.wait_ready();r.hand(False);r.move([.38,0,.55])
 for close,force in [(False,1.5),(True,1.5),(True,3.5),(False,.3)]:
  r.hand(close,force);r.sleep(1);start=time.monotonic();r.sleep(2);a=np.array([f for t,f in r.forces if t>=start]);med=np.median(a,axis=0);mad=np.median(np.abs(a-med),axis=0);good=np.all(np.abs(a-med)<np.maximum(.01,6*mad),axis=1)
  row=dict(close=close,force=force,mean=a.mean(0).tolist(),median=med.tolist(),std=a.std(0).tolist(),quantiles=np.percentile(a,[5,25,75,95],axis=0).tolist(),filtered_std=a[good].std(0).tolist(),good_fraction=float(good.mean()),joints=r.state,velocity=r.velocity);rows.append(row);print(json.dumps(row),flush=True)
  (P/'evidence/wrist-diagnostic.json').write_text(json.dumps(rows,indent=2))
finally:rclpy.shutdown();r.thread.join(3)
