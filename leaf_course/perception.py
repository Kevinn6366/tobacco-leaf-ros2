"""RGB-D segmentation/reprojection only. Does not read simulator model poses."""
import json,cv2,numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from orientation import ridge_segments,direction_at
C=json.loads(Path(__file__).with_name('camera.json').read_text())
def hand_camera():
 T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',C['hand_rpy']).as_matrix();T[:3,3]=C['hand_xyz'];return T

def reproject(depth,T,step=1):
 v,u=np.indices(depth.shape);z=depth.astype(float)
 # Gazebo camera axes: forward x, left y, up z. Depth stores optical axial depth.
 xyz=np.stack([z,-(u-C['cx'])*z/C['fx'],-(v-C['cy'])*z/C['fy']],axis=-1)
 return xyz@T[:3,:3].T+T[:3,3]

def segment(rgb):
 hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
 mask=cv2.inRange(hsv,np.array([6,65,30]),np.array([28,255,250]))
 mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
 n,labels,stats,cents=cv2.connectedComponentsWithStats(mask)
 return mask,n,labels,stats,cents

def detect(rgb,depth,world_hand,region="pile"):
 assert rgb.shape[:2]==depth.shape==(480,640),'Camera shape mismatch'
 mask,n,labels,stats,cents=segment(rgb)
 world=reproject(depth,world_hand@hand_camera());valid=np.isfinite(depth)&(depth>.04)&(depth<2.5)
 out=[];annotated=rgb.copy()
 for i in range(1,n):
  x,y,w,h,area=stats[i]
  if area<80:continue
  sel=(labels==i)&valid
  pts=world[sel]
  if len(pts)<50:continue
  center=np.median(pts,axis=0)
  # Physical workspace gate, not object truth.
  bounds=(.26,.75,.04,.46,.0003,.15) if region=='pile' else (.31,.67,-.43,-.13,.008,.20)
  if not (bounds[0]<center[0]<bounds[1] and bounds[2]<center[1]<bounds[3] and bounds[4]<center[2]<bounds[5]):continue
  xy=pts[:,:2];_,_,vt=np.linalg.svd(xy-np.median(xy,axis=0),full_matrices=False);axis=vt[0]
  if axis[0]<0:axis=-axis
  along=(xy-center[:2])@axis;lo,hi=np.percentile(along,[2,98]);stem=pts[along<lo+.012]
  grasp=np.median(stem,axis=0);grasp[:2]-=axis*.005
  grasp[2]=float(np.percentile(stem[:,2],85))
  pxarea=np.square(depth[sel]/C['fx']);height=np.maximum(pts[:,2]-C['belt_z'],.0015)
  volume=float(np.sum(pxarea*height));mass=volume*230*1000 # calibrated loose bulk density kg/m3 (assumption)
  out.append(dict(center=center.tolist(),grasp=grasp.tolist(),yaw=float(np.arctan2(axis[1],axis[0])),pixels=int(area),points=len(pts),volume_m3=volume,estimated_mass_g=float(np.sum(pxarea)*3300),volume_mass_g=mass,projected_area_m2=float(np.sum(pxarea)),height_mm=float(np.percentile(height,90)*1000),bbox=[int(x),int(y),int(w),int(h)],clipped=bool(x<3 or y<3 or x+w>637 or y+h>477)))
  cv2.rectangle(annotated,(x,y),(x+w,y+h),(30,240,160),2);cv2.putText(annotated,f'G{len(out)} {len(pts)} pts',(x,y-5),cv2.FONT_HERSHEY_SIMPLEX,.45,(30,240,160),1)
 out.sort(key=lambda c:(c['clipped'],abs(c['center'][1]-.23)))
 if region=="pile":
  out=heap_bites(world,mask);T=world_hand@hand_camera();segments=ridge_segments(rgb,world,mask)
  for candidate in out:
   candidate['yaw'],line=direction_at(candidate,segments);candidate['orientation_line_pixels']=line
   if line is not None:cv2.line(annotated,tuple(line[:2]),tuple(line[2:]),(70,180,255),1)
  # Several heap strips may see the same ridge; retain one physical grasp point.
  unique=[]
  for candidate in out:
   if any(np.linalg.norm(np.array(candidate['grasp'])-c['grasp'])<.008 for c in unique):continue
   unique.append(candidate)
  out=unique
  for k,c in enumerate(out):
   pc=T[:3,:3].T@(np.array(c['grasp'])-T[:3,3])
   if pc[0]<=.03:continue
   u=int(C['cx']-C['fx']*pc[1]/pc[0]);v=int(C['cy']-C['fy']*pc[2]/pc[0])
   if 0<=u<640 and 0<=v<480:
    cv2.circle(annotated,(u,v),5,(255,210,40),2);cv2.putText(annotated,str(k+1),(u+7,v),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,210,40),1)
 return out,world,mask,annotated

# Heap boundaries are not leaf instances: generate local bites from visible cloud.
def heap_bites(world,mask):
 valid=(mask>0)&np.all(np.isfinite(world),axis=-1)
 pts=world[valid];pts=pts[(pts[:,0]>.28)&(pts[:,0]<.73)&(pts[:,1]>.04)&(pts[:,1]<.46)&(pts[:,2]>.0003)&(pts[:,2]<.15)]
 if len(pts)<100:return []
 out=[]
 for y in np.arange(np.percentile(pts[:,1],8),np.percentile(pts[:,1],92),.022):
  local=pts[np.abs(pts[:,1]-y)<.017]
  if len(local)<150:continue
  x=np.percentile(local[:,0],4);edge=local[local[:,0]<x+.017]
  if len(edge)<30:continue
  # Fit the exposed boundary over neighboring strips; inward normal follows the leaf edge.
  border=[]
  for yy in np.arange(y-.024,y+.025,.004):
   strip=pts[np.abs(pts[:,1]-yy)<.003]
   if len(strip)>25:border.append([yy,float(np.percentile(strip[:,0],4))])
  slope=float(np.polyfit(np.array(border)[:,0],np.array(border)[:,1],1)[0]) if len(border)>5 else 0.
  yaw=float(np.clip(np.arctan2(-slope,1.),-.75,.75))
  # Pinch an interior fold, not just the outer leaf boundary.
  interior=local[(local[:,0]>np.percentile(local[:,0],35))&(local[:,0]<np.percentile(local[:,0],60))]
  # XYZ must belong to one observed patch; independent medians can land in an air gap.
  center_xy=np.median(interior[:,:2],axis=0);top=float(np.percentile(interior[:,2],70))
  upper=interior[interior[:,2]>=top-.003]
  chosen=upper[np.argmin(np.linalg.norm(upper[:,:2]-center_xy,axis=1))]
  neighbors=interior[(np.linalg.norm(interior[:,:2]-chosen[:2],axis=1)<.005)&(abs(interior[:,2]-chosen[2])<.003)]
  if len(neighbors)<8:continue
  g=np.median(neighbors,axis=0).tolist();top=g[2]
  yaw=0. # Mid-lamina pinch; geometry no longer requires alignment with the outer edge.
  extent=np.percentile(local[:,0],95)-x;area=float(extent*.034)
  out.append(dict(center=np.median(local,axis=0).tolist(),grasp=g,yaw=yaw,pixels=len(local),points=len(local),volume_m3=area*max(.0015,float(np.median(local[:,2]))),projected_area_m2=area,estimated_mass_g=area*3300,height_mm=top*1000,bbox=[0,0,0,0],clipped=False))
 return out
