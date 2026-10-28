"""ROS I/O, trajectory execution and effort-limited fingers. No grasp attachments."""
import time,threading,json,traceback
from collections import deque
from pathlib import Path
import numpy as np,cv2,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState,Image,PointCloud2
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64MultiArray,Float64,String,Header
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation
from kinematics import ik,fk
from perception import detect,C
from stereo import StereoDepth
from height_regions import split_height
P=Path(__file__).resolve().parent
ARM=[f'panda_joint{i}' for i in range(1,8)]
class Robot(Node):
 def __init__(self):
  super().__init__('leaf_course',parameter_overrides=[Parameter('use_sim_time',Parameter.Type.BOOL,True)]);self.state={};self.velocity={};self.hand_target=.04;self.hand_force=1.5;self.hand_gain=250.;self.frames={};self.frame_history={'rgb':deque(maxlen=12),'right':deque(maxlen=12)};self.poses={};self.forces=[];self.wrist_loads=[];self.wrist_q=0.;self.wrist_last=0.;self.scale_forces=[];self.scale_load=0.;self.scale_last=0.;self.contacts={};self.encoder=None;self.cmd_effort=[1.,1.];self.status='IDLE';self.results={};self.bridge=CvBridge();self.subs=[];self.t0=time.monotonic();self.cloud=None;self.candidates=[];self.annotated=None;self.busy=False;self.simtime=0.;self.stop_requested=False;self.active_goal=None;self.box_pose=None;self.box_stamp=0.
  self.scale_trace=[];self.scale_trace_last=-1.
  self.actual_hand=None;self.actual_hand_stamp=0.;self.stereo=StereoDepth(C);self.stereo_quality=0.;self.disparity=None
  self.manifest=json.loads((P/'scene_manifest.json').read_text())
  self.subs.append(self.create_subscription(Clock,'/clock',lambda m:setattr(self,'simtime',m.clock.sec+m.clock.nanosec/1e9),10))
  self.subs.append(self.create_subscription(JointState,'/scale/joints',self.scale_joint,100))
  self.create_timer(.05,self.scale_tick)
  self.passive_pub=self.create_publisher(JointState,'/joint_states',10)
  self.subs.append(self.create_subscription(JointState,'/panda/wrist/loadcell',self.wrist_joint,100))
  self.subs+=[self.create_subscription(JointState,'/joint_states',self.joints,20),self.create_subscription(WrenchStamped,'/panda/wrist/wrench',self.ft,100)]
  for key,topic,fmt in [('rgb','/wrist/stereo/left/image','rgb8'),('right','/wrist/stereo/right/image','rgb8')]:
   self.subs.append(self.create_subscription(Image,topic,lambda m,k=key,f=fmt:self.frame(m,k,f),10))
  self.subs.append(self.create_subscription(TFMessage,'/model/ur/pose',self.robot_pose,10))
  self.subs.append(self.create_subscription(TFMessage,'/model/box1/pose',self.box_callback,10))
  for name in self.manifest:self.subs.append(self.create_subscription(TFMessage,'/model/'+name+'/pose',lambda m,k=name:self.poses.update({k:{t.child_frame_id:t.transform for t in m.transforms}}),10))
  for side in ['left','right']:self.subs.append(self.create_subscription(Contacts,f'/world/leaf_course/model/ur/link/panda_{side}finger/sensor/{side}_contact/contact',lambda m,k=side:self.contacts.update({k:(time.monotonic(),m)}),20))
  self.subs.append(self.create_subscription(Odometry,'/model/conveyor/link/body/odometry',lambda m:setattr(self,'encoder',m),20))
  self.gripper=self.create_publisher(Float64MultiArray,'/panda_hand_controller/commands',10);self.belt_pub=self.create_publisher(Float64,'/model/conveyor/link/body/track_cmd_vel',10)
  self.depth_pub=self.create_publisher(Image,'/wrist/stereo/depth',10);self.disparity_pub=self.create_publisher(Image,'/wrist/stereo/disparity',10)
  self.cloud_pub=self.create_publisher(PointCloud2,'/course/points',10);self.status_pub=self.create_publisher(String,'/course/status',10)
  self.create_timer(.004,self.hand_servo)
  self.create_timer(.5,lambda:self.status_pub.publish(String(data=self.status)))
  self.arm_client=ActionClient(self,FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory')
  self.thread=threading.Thread(target=lambda:rclpy.spin(self),daemon=True);self.thread.start()
 def wrist_joint(self,m):
  for name,q in zip(m.name,m.position):
   if name.endswith('panda_hand_joint'):
    self.wrist_q=q;self.wrist_last=time.monotonic();self.wrist_loads.append((self.wrist_last,np.array([0.,0.,-20000.*q])))
    js=JointState();js.header=m.header;js.name=['panda_hand_joint'];js.position=[q];self.passive_pub.publish(js)
  if len(self.wrist_loads)>5000:del self.wrist_loads[:2500]
 def robot_pose(self,m):
  transforms={t.child_frame_id:t.transform for t in m.transforms};root=transforms.get('ur');hand=next((t for key,t in transforms.items() if key in ['panda_hand','ur/panda_hand','ur::panda_hand']),None)
  if hand is None:return
  def matrix(t):
   T=np.eye(4);T[:3,:3]=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix();T[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return T
  self.actual_hand=(matrix(root) if root is not None else np.eye(4))@matrix(hand);self.actual_hand_stamp=time.monotonic()
 def hand_pose(self):
  if self.actual_hand is not None and time.monotonic()-self.actual_hand_stamp<2:return self.actual_hand.copy()
  T=fk(self.q());T[:3,3]+=T[:3,2]*self.wrist_q;return T
 def box_callback(self,m):
  for t in m.transforms:
   if t.child_frame_id=="box1":self.box_pose=t.transform;self.box_stamp=time.monotonic()
 def box_xyz(self):
  if self.box_pose is None or time.monotonic()-self.box_stamp>3:raise RuntimeError("Box pose feedback unavailable")
  p=self.box_pose.translation;return np.array([p.x,p.y,p.z])
 def check_stop(self):
  if self.stop_requested:
   if self.active_goal is not None:self.active_goal.cancel_goal_async()
   raise InterruptedError("已停止：复位后可重新开始")
 def scale_joint(self,m):
  for name,q in zip(m.name,m.position):
   if name.endswith('loadcell'):self.scale_load=-1500.*q;self.scale_last=time.monotonic()
 def scale_contact(self,m):
  force=0.
  for c in m.contacts:
   first=c.collision1.name.startswith('scale::')
   for wrench in c.wrenches:
    f=wrench.body_1_wrench.force if first else wrench.body_2_wrench.force
    force+=f.z
  self.scale_load=abs(force);self.scale_last=time.monotonic()
 def scale_tick(self):
  if self.simtime-self.scale_trace_last>=.1 and time.monotonic()-self.scale_last<1:
   self.scale_trace.append([self.simtime,self.scale_load]);self.scale_trace_last=self.simtime
  self.scale_forces.append((time.monotonic(),self.scale_load if time.monotonic()-self.scale_last<1 else 0.))
  if len(self.scale_forces)>3000:del self.scale_forces[:1500]
 def joints(self,m):
  self.state.update(zip(m.name,m.position));self.velocity.update(zip(m.name,m.velocity))
 def hand_servo(self):
  efforts=[float(np.clip(self.hand_gain*(self.hand_target-self.state.get(j,.04))-3*self.velocity.get(j,0),-self.hand_force,self.hand_force)) for j in ("panda_finger_joint1","panda_finger_joint2")]
  self.gripper.publish(Float64MultiArray(data=efforts))
 def frame(self,m,k,f):
  item=(time.monotonic(),m.header,self.bridge.imgmsg_to_cv2(m,f));self.frames[k]=item;self.frame_history[k].append(item)
 def ft(self,m):
  self.forces.append((time.monotonic(),np.array([m.wrench.force.x,m.wrench.force.y,m.wrench.force.z])))
  if len(self.forces)>3000:del self.forces[:1500]
 def say(self,s):self.status=s;print(s,flush=True)
 def wait_ready(self):
  deadline=time.monotonic()+120
  while time.monotonic()<deadline:
   if len(self.state)>=9 and all(k in self.frames for k in ['rgb','right']) and self.arm_client.wait_for_server(timeout_sec=1):return
   time.sleep(.2)
  raise RuntimeError('Robot/camera/controller not ready')
 def q(self):return np.array([self.state[j] for j in ARM])
 def move(self,xyz,duration=3,yaw=0.,sideways=False,linear=False,roll=0.):
  self.check_stop();xyz=np.array(xyz);seed=self.q();start=self.hand_pose()[:3,3].copy()
  # A scan is a historical observation, not moving material. Remove stale RViz points on motion.
  self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='world',stamp=self.get_clock().now().to_msg()),np.empty((0,3),dtype=np.float32)))
  # Close to material, preserve the tool orientation and a straight Cartesian path.
  count=max(2,int(np.ceil(np.linalg.norm(xyz-start)/.012))) if linear else 1
  desired_R=Rotation.from_euler('z',yaw).as_matrix()@(np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]) if sideways else np.diag([1.,-1.,-1.]))@Rotation.from_euler('x',roll).as_matrix()
  g=FollowJointTrajectory.Goal();g.trajectory.joint_names=ARM
  for k in range(1,count+1):
   fraction=k/count;position=start+(xyz-start)*fraction if linear else xyz
   seed=ik(position-desired_R[:,2]*self.wrist_q,seed,yaw=yaw,sideways=sideways,roll=roll)
   point=JointTrajectoryPoint();point.positions=seed.tolist();seconds=float(duration)*fraction
   point.time_from_start.sec=int(seconds);point.time_from_start.nanosec=int((seconds-int(seconds))*1e9);g.trajectory.points.append(point)
  f=self.arm_client.send_goal_async(g);self.wait_future(f);h=f.result();assert h and h.accepted,'Arm goal rejected';self.active_goal=h;res=h.get_result_async();self.wait_future(res,180);r=res.result();assert r.result.error_code==0,str(r)
  self.sleep(.25);actual=self.hand_pose();expected=Rotation.from_euler('z',yaw).as_matrix()@(np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]]) if sideways else np.diag([1.,-1.,-1.]))@Rotation.from_euler('x',roll).as_matrix()
  # Pose messages can lag the action result under heavy simulation load.
  # Wait for convergence while retaining the same physical error limits.
  settle_deadline=time.monotonic()+20
  while (np.linalg.norm(actual[:3,3]-xyz)>.003 or Rotation.from_matrix(expected@actual[:3,:3].T).magnitude()>.025) and time.monotonic()<settle_deadline:
   self.check_stop();time.sleep(.05);actual=self.hand_pose()
  if self.actual_hand is None or time.monotonic()-self.actual_hand_stamp>2:raise RuntimeError('夹爪物理位姿反馈失联，禁止用正运动学代替验收')
  if np.linalg.norm(actual[:3,3]-xyz)>.003 or Rotation.from_matrix(expected@actual[:3,:3].T).magnitude()>.025:raise RuntimeError(f'夹爪实际位姿未到位: 位置误差 {np.linalg.norm(actual[:3,3]-xyz)*1000:.1f} mm（XYZ={(1000*(actual[:3,3]-xyz)).round(2).tolist()}），姿态误差 {Rotation.from_matrix(expected@actual[:3,:3].T).magnitude():.3f} rad')
  return actual[:3,3].tolist()
 def wait_future(self,f,timeout=30):
  end=time.monotonic()+timeout
  while not f.done() and time.monotonic()<end:self.check_stop();time.sleep(.02)
  if not f.done():raise TimeoutError('ROS action timed out')
 def sleep(self,seconds):
  end=self.simtime+seconds;deadline=time.monotonic()+max(120,seconds*80)
  while self.simtime<end:
   self.check_stop()
   if time.monotonic()>deadline:raise TimeoutError("Simulation too slow/stopped")
   time.sleep(.025)
 def hand(self,close=False,force=1.5,opening=.114,stiffness=250.,closed_position=.004,close_duration=1.8):
  target=closed_position if close else float(np.clip((opening+.016)/2,.0081,.065));self.hand_force=force;self.hand_gain=stiffness
  if close:
   start=self.hand_target
   steps=max(20,int(np.ceil(close_duration/.05)))
   for step in range(1,steps+1):
    u=step/steps;blend=u*u*(3-2*u)
    self.hand_target=start+(target-start)*blend;self.sleep(close_duration/steps)
   self.sleep(.4)
  else:self.hand_target=target;self.sleep(1.2)
 def belt(self,v):self.belt_pub.publish(Float64(data=float(v)))
 def sense(self,region="pile"):
  deadline=time.monotonic()+12
  def stamp(item):return item[1].stamp.sec+item[1].stamp.nanosec/1e9
  pair=None
  while time.monotonic()<deadline:
   self.check_stop()
   pairs=[(a,b) for a in list(self.frame_history['rgb']) for b in list(self.frame_history['right']) if abs(stamp(a)-stamp(b))<.03 and time.monotonic()-min(a[0],b[0])<4 and self.simtime-min(stamp(a),stamp(b))<.35]
   if pairs:pair=max(pairs,key=lambda ab:min(stamp(ab[0]),stamp(ab[1])));break
   time.sleep(.02)
  if pair is None:raise RuntimeError('没有新鲜同步双目 RGB 图像，禁止规划')
  a,b=pair
  left,right,depth,disparity,valid=self.stereo.compute(a[2],b[2]);self.observation_rgb=left.copy();self.stereo_quality=float(np.mean(valid));self.disparity=disparity;self.frames['depth']=(time.monotonic(),a[1],depth)
  for publisher,array in [(self.depth_pub,depth),(self.disparity_pub,disparity)]:
   message=self.bridge.cv2_to_imgmsg(array.astype(np.float32),'32FC1');message.header=a[1];publisher.publish(message)
  candidates,world,mask,ann=detect(left,depth,self.hand_pose(),region=region)
  for candidate in candidates:candidate['image_sim_s']=stamp(a)
  self.height_labels,self.height_regions=split_height(left,world,mask)
  self.cloud_sim_s=stamp(a);self.candidates=candidates;self.cloud_hand=self.hand_pose();self.cloud=world;self.annotated=ann
  foreground_points=world[(mask>0)&np.all(np.isfinite(world),axis=-1)]
  self.valid_cloud_points=len(foreground_points);pts=foreground_points[::3]
  self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='world',stamp=a[1].stamp),pts.astype(np.float32)))
  return candidates
 def sample(self):
  for attempt in range(4):
   self.sleep(1);start=time.monotonic();self.sleep(1);a=np.array([f for t,f in self.wrist_loads if t>=start]);assert len(a)>20 and time.monotonic()-self.wrist_last<1,'Missing wrist load cell'
   median=np.median(a,axis=0);mad=np.median(np.abs(a-median),axis=0);good=np.all(np.abs(a-median)<np.maximum(.01,6*mad),axis=1);filtered=a[good]
   if len(filtered)>.8*len(a) and np.linalg.norm(np.std(filtered,axis=0))<.025:return np.mean(filtered,axis=0),np.std(filtered,axis=0)
  raise RuntimeError('腕部读数持续不稳定，禁止把动态冲击当成夹持重量')
 def scale_sample(self):
  for attempt in range(4):
   self.sleep(.8);start=time.monotonic();self.sleep(.8);a=np.array([f for t,f in self.scale_forces if t>=start])
   if len(a)>15 and time.monotonic()-self.scale_last<1 and np.std(a)<.005:return float(np.mean(a)),float(np.std(a))
  raise RuntimeError('盒下秤读数未稳定或已失联，禁止复核与交付')
 def truth(self):
  out={}
  for name,trans in self.poses.items():
   t=trans.get(name)
   if t is None:continue
   R=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);base=np.array([t.translation.x,t.translation.y,t.translation.z]);parts=[]
   for part in self.manifest[name]['parts']:
    pp=next((v for k,v in trans.items() if k in (name+'/'+part['link'],name+'::'+part['link'],part['link'])),None)
    if pp is not None:
     pos=base+R.apply([pp.translation.x,pp.translation.y,pp.translation.z]);rot=R*Rotation.from_quat([pp.rotation.x,pp.rotation.y,pp.rotation.z,pp.rotation.w]);parts.append({'xyz':pos.tolist(),'quat':rot.as_quat().tolist()})
   if parts:out[name]=parts
  return out
 def contact_names(self):
  d={}
  for k,(t,m) in self.contacts.items():
   d[k]=[c.collision1.name+' | '+c.collision2.name for c in m.contacts] if time.monotonic()-t<2 else []
  return d
 def snapshot(self,name):
  deadline=time.monotonic()+15
  while len(self.truth())<len(self.manifest) and time.monotonic()<deadline:
   self.check_stop();time.sleep(.05)
  if len(self.truth())<len(self.manifest):raise RuntimeError('物理记录缺少烟叶位姿，不能保存完整关键帧')
  p=P/'evidence';p.mkdir(exist_ok=True)
  if 'rgb' in self.frames:cv2.imwrite(str(p/(name+'-rgb.png')),cv2.cvtColor(self.frames['rgb'][2],cv2.COLOR_RGB2BGR))
  if 'right' in self.frames:cv2.imwrite(str(p/(name+'-right-rgb.png')),cv2.cvtColor(self.frames['right'][2],cv2.COLOR_RGB2BGR))
  if self.disparity is not None:np.save(p/(name+'-disparity.npy'),self.disparity)
  if self.cloud is not None:
   np.savez_compressed(p/(name+'-cloud.npz'),xyz=self.cloud,world_hand=self.cloud_hand,sim_time_s=self.cloud_sim_s)
   cv2.imwrite(str(p/(name+'-observed-rgb.png')),cv2.cvtColor(self.observation_rgb,cv2.COLOR_RGB2BGR))
  if getattr(self,'height_labels',None) is not None:
   np.save(p/(name+'-height-labels.npy'),self.height_labels)
   (p/(name+'-height-regions.json')).write_text(json.dumps(self.height_regions,indent=2))
  if self.annotated is not None:cv2.imwrite(str(p/(name+'-detection.png')),cv2.cvtColor(self.annotated,cv2.COLOR_RGB2BGR))
  if 'depth' in self.frames:
   d=self.frames['depth'][2];np.save(p/(name+'-depth.npy'),d);vis=cv2.applyColorMap(np.uint8(np.nan_to_num(np.clip((d-.2)/.8,0,1))*255),cv2.COLORMAP_TURBO);cv2.imwrite(str(p/(name+'-depth.png')),vis)
  (p/(name+'-physics.json')).write_text(json.dumps(dict(sim_time_s=self.simtime,camera_sim_s=None if 'rgb' not in self.frames else self.frames['rgb'][1].stamp.sec+self.frames['rgb'][1].stamp.nanosec/1e9,parts=self.truth(),contacts=self.contact_names(),joints=self.state,box_xyz=None if self.box_pose is None else self.box_xyz().tolist()),indent=2))
