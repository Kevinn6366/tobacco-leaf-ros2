"""Isolated stereo display service. Reads RGB; publishes visualization only.
No Robot instance, action clients, command publishers, or simulator pose access.
"""
import json,time,threading,copy
from pathlib import Path
from collections import OrderedDict
import cv2,numpy as np,rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
from stereo import StereoDepth
cv2.setNumThreads(1)
class ConsoleStereo(Node):
 def __init__(self):
  super().__init__('console_stereo_display')
  self.config=json.loads((Path(__file__).parent/'live_mechanics/camera.json').read_text())
  self.stereo=StereoDepth(self.config);self.bridge=CvBridge();self.buffers=[OrderedDict(),OrderedDict()];self.lock=threading.Lock();self.pending=None;self.last_stamp=[None,None];self.count=0
  self.depth_pub=self.create_publisher(Image,'/console/stereo/depth',qos_profile_sensor_data)
  self.cloud_pub=self.create_publisher(PointCloud2,'/console/stereo/points',qos_profile_sensor_data)
  self.info_pub=self.create_publisher(String,'/console/stereo/status',10)
  for i,side in enumerate(['left','right']):self.create_subscription(Image,f'/wrist/stereo/{side}/image',lambda m,i=i:self.receive(m,i),20)
  threading.Thread(target=self.work,daemon=True).start()
 def receive(self,m,i):
  stamp=m.header.stamp.sec*10**9+m.header.stamp.nanosec
  with self.lock:
   if self.last_stamp[i] is not None and stamp<self.last_stamp[i]-10**9:self.buffers=[OrderedDict(),OrderedDict()];self.pending=None
   self.last_stamp[i]=stamp;self.buffers[i][stamp]=m
   while len(self.buffers[i])>40:self.buffers[i].popitem(last=False)
   common=self.buffers[0].keys()&self.buffers[1].keys()
   if common:
    s=max(common);self.pending=(self.buffers[0][s],self.buffers[1][s])
    for b in self.buffers:
     for k in list(b):
      if k<=s:del b[k]
 def work(self):
  while rclpy.ok():
   time.sleep(.5)
   with self.lock:pair=self.pending;self.pending=None
   if pair is None:continue
   try:
    started=time.monotonic();left,right=[self.bridge.imgmsg_to_cv2(m,'rgb8') for m in pair]
    _,_,depth,_,valid=self.stereo.compute(left,right)
    header=copy.deepcopy(pair[0].header);header.frame_id='leaf_left_rectified_optical'
    msg=self.bridge.cv2_to_imgmsg(depth,encoding='32FC1');msg.header=header;self.depth_pub.publish(msg)
    # Optical axes: +x right, +y down, +z forward. Invalid pixels stay absent.
    v,u=np.indices(depth.shape);sel=valid.copy();sel[1::2,:]=False;sel[:,1::2]=False
    z=depth[sel];c=self.config
    xyz=np.column_stack(((u[sel]-c['cx'])*z/c['fx'],(v[sel]-c['cy'])*z/c['fy'],z)).astype(np.float32)
    self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header,xyz))
    self.count+=1
    info=dict(pairs=self.count,stamp_s=header.stamp.sec+header.stamp.nanosec/1e9,sync_delta_ms=0,valid_pct=round(float(valid.mean()*100),2),points=len(xyz),compute_ms=round((time.monotonic()-started)*1000,1),source='stereo RGB / SGBM',frame=header.frame_id)
    self.info_pub.publish(String(data=json.dumps(info)))
    if self.count%10==1:print(json.dumps(info),flush=True)
   except Exception as e:self.get_logger().error(str(e))
def main():
 rclpy.init();node=ConsoleStereo()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
