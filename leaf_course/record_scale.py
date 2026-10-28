"""Read-only independent recorder for the physical box scale during a batch."""
from pathlib import Path
import json,time,rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
P=Path(__file__).resolve().parent
rclpy.init();n=Node('course_scale_recorder');samples=[];clock=[0.];last=[-1.];tare=None
n.create_subscription(Clock,'/clock',lambda m:clock.__setitem__(0,m.clock.sec+m.clock.nanosec/1e9),10)
def on_scale(m):
 for name,q in zip(m.name,m.position):
  if name.endswith('loadcell') and clock[0]-last[0]>=.1:
   samples.append([clock[0],-1500*q]);last[0]=clock[0]
n.create_subscription(JointState,'/scale/joints',on_scale,100)
def save():
 (P/'evidence/scale-trace-observer.json').write_text(json.dumps(dict(columns=['sim_s','force_N'],tare_N=tare,samples=samples,source='independent read-only ROS scale joint recorder')))
started=time.monotonic();check=0.
try:
 while rclpy.ok() and time.monotonic()-started<7200:
  rclpy.spin_once(n,timeout_sec=.1)
  if time.monotonic()-check>3:
   s=json.loads((P/'status.json').read_text());tare=s.get('results',{}).get('scale_tare_N');save();check=time.monotonic()
   if not s.get('busy') and time.monotonic()-started>10:break
finally:save();n.destroy_node();rclpy.shutdown()
