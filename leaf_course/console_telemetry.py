"""Read-only telemetry for host console. Does not publish commands."""
import json,time,threading,math
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy,cv2,numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,JointState,PointCloud2
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from ros_gz_interfaces.msg import Contacts
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2
values={};frames={};bridge=CvBridge();last={}
def put(k,v):values[k]={'value':v,'at':time.monotonic()}
def picture(m,k):
 if time.monotonic()-last.get(k,0)<.5:return
 last[k]=time.monotonic()
 try:
  a=bridge.imgmsg_to_cv2(m,'passthrough' if k=='depth' else 'bgr8')
  if k=='depth':
   a=a.astype(float);a=a/1000 if m.encoding=='16UC1' else a
   good=np.isfinite(a)&(a>0);out=np.zeros(a.shape,dtype=np.uint8);out[good]=np.clip(a[good]/2.5*255,0,255).astype(np.uint8);a=cv2.applyColorMap(out,cv2.COLORMAP_TURBO);a[~good]=0
  ok,b=cv2.imencode('.jpg',a)
  if ok:frames[k]=b.tobytes();put(k,True)
 except Exception:pass
def joint(m,k,suffix,stiff):
 for n,q in zip(m.name,m.position):
  if n.endswith(suffix):put(k,{'force_n':-stiff*q,'displacement_mm':q*1000})
def contact(m,k):
 leaf=[c for c in m.contacts if 'leaf_' in c.collision1.name+' '+c.collision2.name]
 put(k,{'count':len(m.contacts),'leaf_count':len(leaf)})
def cloud(m):
 if time.monotonic()-last.get('cloud',0)<1:return
 last['cloud']=time.monotonic()
 try:
  a=np.array(list(point_cloud2.read_points_list(m,field_names=('x','y','z'),skip_nans=True)),dtype=float).reshape(-1,3)
  a=a[np.isfinite(a).all(axis=1)];n=len(a);a=a[::max(1,math.ceil(n/1600))]
  put('cloud',{'points':a.tolist(),'count':n,'frame':m.header.frame_id})
 except Exception:pass
rclpy.init();node=Node('host_console_readonly');q=qos_profile_sensor_data
for k in ['left','right']:node.create_subscription(Image,f'/wrist/stereo/{k}/image',lambda m,k=k:picture(m,k),20)
node.create_subscription(Image,'/console/stereo/depth',lambda m:picture(m,'depth'),q)
node.create_subscription(PointCloud2,'/console/stereo/points',cloud,q)
node.create_subscription(String,'/console/stereo/status',lambda m:put('stereo_status',json.loads(m.data)),10)
node.create_subscription(WrenchStamped,'/panda/wrist/wrench',lambda m:put('wrench',{'force':[m.wrench.force.x,m.wrench.force.y,m.wrench.force.z],'torque':[m.wrench.torque.x,m.wrench.torque.y,m.wrench.torque.z]}),q)
node.create_subscription(JointState,'/panda/wrist/loadcell',lambda m:joint(m,'wrist','panda_hand_joint',20000),q)
node.create_subscription(JointState,'/scale/joints',lambda m:joint(m,'scale','loadcell',1500),q)
node.create_subscription(JointState,'/joint_states',lambda m:put('joints',dict(zip(m.name,m.position))) if 'panda_joint1' in m.name else None,q)
for k in ['left','right']:node.create_subscription(Contacts,f'/world/leaf_course/model/ur/link/panda_{k}finger/sensor/{k}_contact/contact',lambda m,k=k:contact(m,k+'_contact'),q)
node.create_subscription(Contacts,'/world/leaf_course/model/scale/link/tray/sensor/scale_contacts/contact',lambda m:contact(m,'tray_contact'),q)
threading.Thread(target=lambda:rclpy.spin(node),daemon=True).start()
class H(BaseHTTPRequestHandler):
 def log_message(self,*a):pass
 def do_GET(self):
  key=self.path.split('?')[0]
  if key=='/telemetry':data=json.dumps({k:{'value':v['value'],'age_s':round(time.monotonic()-v['at'],2)} for k,v in list(values.items())},allow_nan=False).encode();kind='application/json'
  elif key.startswith('/frames/') and key[8:-4] in frames:data=frames[key[8:-4]];kind='image/jpeg'
  else:self.send_error(404);return
  self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)));self.end_headers();self.wfile.write(data)
ThreadingHTTPServer(('127.0.0.1',8083),H).serve_forever()
