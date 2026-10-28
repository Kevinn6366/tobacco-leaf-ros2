"""Separate Gazebo moving-belt/RGB-D/encoder experiment; does not replace the ground-heap scene."""
import os,subprocess,signal,time,json,copy,math,xml.etree.ElementTree as ET
from pathlib import Path
os.environ['ROS_DOMAIN_ID']='37';os.environ['GZ_PARTITION']='leaf_belt_acceptance'
import numpy as np,cv2,rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from motion_compensation import predict_position
P=Path(__file__).resolve().parent;D=Path('/tmp/leaf_belt_acceptance');D.mkdir(exist_ok=True)
def sub(p,t,v=None,**kw):
 x=ET.SubElement(p,t,kw)
 if v is not None:x.text=str(v)
 return x
s=ET.Element('sdf',version='1.9');w=sub(s,'world',name='belt_acceptance');ph=sub(w,'physics',name='physics',type='ignored');sub(ph,'max_step_size',.002);sub(ph,'real_time_factor',1)
for lib,cls in [('physics','Physics'),('user-commands','UserCommands'),('scene-broadcaster','SceneBroadcaster'),('sensors','Sensors')]:
 p=sub(w,'plugin',filename='gz-sim-'+lib+'-system',name='gz::sim::systems::'+cls)
 if cls=='Sensors':sub(p,'render_engine','ogre')
sc=sub(w,'scene');sub(sc,'ambient','.8 .8 .8 1');sun=sub(w,'light',name='sun',type='directional');sub(sun,'direction','0 0 -1');sub(sun,'diffuse','.8 .8 .8 1')
m=sub(w,'model',name='conveyor');sub(m,'static','true');l=sub(m,'link',name='body')
for tag in ['visual','collision']:
 e=sub(l,tag,name=tag);sub(e,'pose','0 0 -.025 0 0 0');sub(sub(sub(e,'geometry'),'box'),'size','2 .8 .05')
 if tag=='visual':mat=sub(e,'material');sub(mat,'diffuse','.14 .18 .22 1');sub(mat,'ambient','.14 .18 .22 1')
 else:fr=sub(sub(sub(e,'surface'),'friction'),'ode');sub(fr,'mu',1.0);sub(fr,'mu2',1.0)
p=sub(m,'plugin',filename='gz-sim-track-controller-system',name='gz::sim::systems::TrackController');sub(p,'link','body');sub(p,'odometry_publish_frequency',50)
leaf=copy.deepcopy(ET.parse(P/'world.sdf').find(".//model[@name='leaf_00']"));leaf.find('pose').text='-.32 0 .01 0 0 .2';w.append(leaf)
m=sub(w,'model',name='camera');sub(m,'static','true');l=sub(m,'link',name='camera');sub(l,'pose','0 0 1 0 1.5707963267948966 0');sen=sub(l,'sensor',name='rgbd',type='rgbd_camera');sub(sen,'always_on','true');sub(sen,'update_rate',8);sub(sen,'topic','/belt_camera');cam=sub(sen,'camera');sub(cam,'horizontal_fov',math.pi/3);im=sub(cam,'image');sub(im,'width',640);sub(im,'height',480);sub(im,'format','R8G8B8');cl=sub(cam,'clip');sub(cl,'near',.05);sub(cl,'far',3)
ET.ElementTree(s).write(D/'world.sdf',encoding='unicode');log=open(D/'sim.log','w');processes=[];rows=[]
try:
 processes.append(subprocess.Popen(['gz','sim','-s','-r','--physics-engine','gz-physics-dartsim-plugin',str(D/'world.sdf')],stdout=log,stderr=log,start_new_session=True))
 args=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/belt_camera/image@sensor_msgs/msg/Image[gz.msgs.Image','/belt_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image','/model/conveyor/link/body/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry','/model/conveyor/link/body/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double']
 processes.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',*args],stdout=log,stderr=log,start_new_session=True))
 rclpy.init();node=Node('belt_acceptance');bridge=CvBridge();state={'time':0.,'velocity':None};frames={}
 node.create_subscription(Clock,'/clock',lambda m:state.update(time=m.clock.sec+m.clock.nanosec/1e9),10)
 node.create_subscription(Odometry,'/model/conveyor/link/body/odometry',lambda m:state.update(velocity=m.twist.twist.linear.x),20)
 def frame(m,key,fmt):frames[key]=(m.header.stamp.sec+m.header.stamp.nanosec/1e9,bridge.imgmsg_to_cv2(m,fmt))
 node.create_subscription(Image,'/belt_camera/image',lambda m:frame(m,'rgb','rgb8'),10);node.create_subscription(Image,'/belt_camera/depth_image',lambda m:frame(m,'depth','32FC1'),10)
 cmd=node.create_publisher(Float64,'/model/conveyor/link/body/track_cmd_vel',10)
 def wait(pred,timeout=90):
  end=time.monotonic()+timeout
  while not pred():
   if time.monotonic()>end:raise TimeoutError('Belt sensor/clock timeout')
   rclpy.spin_once(node,timeout_sec=.03)
 def sleep(dt):
  end=state['time']+dt;wait(lambda:state['time']>=end)
 def observe(after=-1):
  wait(lambda:'rgb' in frames and 'depth' in frames and abs(frames['rgb'][0]-frames['depth'][0])<.01 and frames['rgb'][0]>after)
  stamp,rgb=frames['rgb'];dep=frames['depth'][1];mask=cv2.inRange(cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV),np.array([6,65,30]),np.array([38,255,250]));v,u=np.nonzero((mask>0)&np.isfinite(dep)&(dep>.5)&(dep<1.1))
  if len(u)<100:raise RuntimeError('No valid moving leaf observation')
  z=dep[v,u];fx=640/(2*np.tan(np.pi/6));xyz=np.c_[-(v-239.5)*z/fx,-(u-319.5)*z/fx,1-z]
  return stamp,np.median(xyz,axis=0),rgb
 wait(lambda:state['velocity'] is not None and len(frames)==2);sleep(2.)
 for speed in [.03,.06,.10]:
  cmd.publish(Float64(data=speed));sleep(.7)
  for repeat in range(3):
   t,p,rgb=observe();encoder=state['velocity'];t2,q,rgb2=observe(t+.49);latency=t2-t;pred=predict_position(p,np.array([encoder,0,0]),latency,0)
   row=dict(command_m_s=speed,encoder_m_s=encoder,latency_s=latency,initial_xyz=p.tolist(),later_xyz=q.tolist(),uncompensated_mm=float(np.linalg.norm(q[:2]-p[:2])*1000),compensated_mm=float(np.linalg.norm(q[:2]-pred[:2])*1000));rows.append(row);print(row,flush=True)
   cv2.imwrite(str(P/'evidence'/f'belt-physical-{len(rows)}.png'),cv2.cvtColor(rgb2,cv2.COLOR_RGB2BGR))
 cmd.publish(Float64(data=0.));sleep(.5);node.destroy_node();rclpy.shutdown()
 result=dict(evidence_type='Gazebo physical TrackController conveyor + RGB-D threshold centroid + measured belt odometry; no manipulator in this isolated bench',rows=rows,pass_test=bool(rows) and all(r['compensated_mm']<5 for r in rows))
except Exception as ex:result=dict(error=str(ex),rows=rows,pass_test=False)
finally:
 for proc in processes:
  if proc.poll() is None:
   os.killpg(proc.pid,signal.SIGINT)
   try:proc.wait(timeout=3)
   except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL)
 (P/'evidence/belt-physical.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
