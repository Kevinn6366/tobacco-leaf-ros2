from visualization_msgs.msg import Marker,MarkerArray
from scipy.spatial.transform import Rotation
import numpy as np,json
from runtime import P

def marker(i,pos,size,color,typ=Marker.CUBE):
 m=Marker();m.header.frame_id='world';m.ns='course';m.id=i;m.type=typ;m.action=Marker.ADD;m.pose.orientation.w=1.;m.pose.position.x,m.pose.position.y,m.pose.position.z=map(float,pos);m.scale.x,m.scale.y,m.scale.z=map(float,size);m.color.r,m.color.g,m.color.b,m.color.a=map(float,color);return m
class SceneView:
 def __init__(self,r):
  self.r=r;self.pub=r.create_publisher(MarkerArray,'/leaf_scene',10);r.create_timer(.25,self.render);self.boxparts=json.loads((P/'box_geometry.json').read_text())
 def render(self):
  arr=MarkerArray()
  arr.markers.append(marker(1,[0,0,-.025],[4,5,.05],[.14,.18,.22,1]))
  for k,(x,y,sx,sy,color,label) in enumerate([(.50,.25,.48,.40,[.80,.67,.22,1],'1  LEAF SOURCE'),(.49,-.28,.35,.33,[.25,.65,.92,1],'2  WEIGH BOX'),(.10,-.53,.34,.32,[.20,.75,.40,1],'3  DELIVERY')]):
   for j,(dx,dy,ax,ay) in enumerate([(-sx/2,0,.004,sy),(sx/2,0,.004,sy),(0,-sy/2,sx,.004),(0,sy/2,sx,.004)]):arr.markers.append(marker(30+k*5+j,[x+dx,y+dy,.0003],[ax,ay,.0004],color))
   text=marker(34+k*5,[x,y-sy/2-.025,.008],[0,0,.025],color,Marker.TEXT_VIEW_FACING);text.text=label;arr.markers.append(text)
  if self.r.box_pose:
   t=self.r.box_pose;R=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]);xyz=np.array([t.translation.x,t.translation.y,t.translation.z])
   for i,(_,pos,size) in enumerate(self.boxparts):
    m=marker(10+i,xyz+R.apply(pos),size,[.1,.3,.5,1]);m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=R.as_quat();arr.markers.append(m)
  truth=self.r.truth()
  for idx,(name,spec) in enumerate(self.r.manifest.items()):
   for k,(state,part) in enumerate(zip(truth.get(name,[]),spec['parts'])):
    m=marker(100+idx*10+k,state['xyz'],[1,1,1],part['color'],Marker.MESH_RESOURCE);m.mesh_resource='file://'+str(P/part['mesh']);m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=state['quat'];arr.markers.append(m)
  self.pub.publish(arr)
