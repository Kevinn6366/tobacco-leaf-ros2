"""Offline instance audit of a saved, stationary Gazebo stereo observation.
Model poses are used ONLY to score segmentation, never to guide a grasp.
Rasterized visible mesh labels use a depth buffer; fully hidden leaves remain misses
in all-scene recall, and are separately excluded from visible-instance recall.
"""
from pathlib import Path
import argparse,json
import cv2,numpy as np
from scipy.spatial.transform import Rotation
from perception import C,hand_camera,segment
P=Path(__file__).resolve().parent

def mesh(path):
 vertices=[];faces=[]
 for line in path.read_text().splitlines():
  values=line.split()
  if not values:continue
  if values[0]=='v':vertices.append(list(map(float,values[1:4])))
  elif values[0]=='f':
   indices=[int(x.split('/')[0])-1 for x in values[1:]]
   faces.extend([[indices[0],indices[k],indices[k+1]] for k in range(1,len(indices)-1)])
 return np.array(vertices),np.array(faces)

def raster_triangle(uv,z,identity,depth,labels):
 lo=np.maximum(np.floor(uv.min(0)).astype(int),[0,0]);hi=np.minimum(np.ceil(uv.max(0)).astype(int),[labels.shape[1]-1,labels.shape[0]-1])
 if np.any(hi<lo):return
 x,y=np.meshgrid(np.arange(lo[0],hi[0]+1)+.5,np.arange(lo[1],hi[1]+1)+.5)
 a,b,c=uv;den=(b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
 if abs(den)<1e-8:return
 w0=((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1]))/den
 w1=((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1]))/den;w2=1-w0-w1
 inside=(w0>=0)&(w1>=0)&(w2>=0)
 inverse=w0/z[0]+w1/z[1]+w2/z[2];d=np.divide(1,inverse,out=np.full_like(inverse,np.inf),where=inverse>0)
 region=np.s_[lo[1]:hi[1]+1,lo[0]:hi[0]+1];closer=inside&(d<depth[region]);depth[region][closer]=d[closer];labels[region][closer]=identity

def evaluate(prefix):
 evidence=P/'evidence';cloud=np.load(evidence/(prefix+'-cloud.npz'));state=json.loads((evidence/(prefix+'-physics.json')).read_text());manifest=json.loads((P/'scene_manifest.json').read_text())
 image_path=evidence/(prefix+'-observed-rgb.png')
 if not image_path.exists():image_path=evidence/(prefix+'-rgb.png')
 rgb=cv2.cvtColor(cv2.imread(str(image_path)),cv2.COLOR_BGR2RGB)
 T=cloud['world_hand']@hand_camera();labels=np.zeros(rgb.shape[:2],np.int32);depth=np.full(labels.shape,np.inf)
 names=sorted(state['parts'])
 for identity,name in enumerate(names,1):
  for part,pose in zip(manifest[name]['parts'],state['parts'][name]):
   v,faces=mesh(P/part['mesh']);world=Rotation.from_quat(pose['quat']).apply(v)+pose['xyz'];cam=(world-T[:3,3])@T[:3,:3]
   uv=np.c_[C['cx']-C['fx']*cam[:,1]/cam[:,0],C['cy']-C['fy']*cam[:,2]/cam[:,0]]
   for face in faces:
    if np.all(cam[face,0]>.03):raster_triangle(uv[face],cam[face,0],identity,depth,labels)
 mask,n,pred,stats,_=segment(rgb)
 # Ground-heap ROI, determined from calibrated measured cloud, not truth labels.
 xyz=cloud['xyz'];roi=np.all(np.isfinite(xyz),axis=-1)&(xyz[:,:,0]>.26)&(xyz[:,:,0]<.75)&(xyz[:,:,1]>.04)&(xyz[:,:,1]<.46)&(xyz[:,:,2]>.0003)&(xyz[:,:,2]<.15)
 ids=[i for i in range(1,n) if stats[i,4]>=80 and np.count_nonzero((pred==i)&roi)>=50]
 pairs=[];visible=[i for i in range(1,len(names)+1) if np.count_nonzero(labels==i)>=80]
 for gt in visible:
  a=labels==gt
  for detection in ids:
   b=pred==detection;iou=float(np.count_nonzero(a&b)/max(1,np.count_nonzero(a|b)))
   if iou>=.5:pairs.append((iou,gt,detection))
 matched_gt=set();matched_pred=set();matches=[]
 for iou,g,p in sorted(pairs,reverse=True):
  if g not in matched_gt and p not in matched_pred:matched_gt.add(g);matched_pred.add(p);matches.append(dict(leaf=names[g-1],region=p,iou=iou))
 tp=len(matches);result=dict(evidence_type='offline Gazebo mesh projection audit of actual stereo RGB; stationary observation required',scene_leaves=len(names),visible_leaves_ge80pixels=len(visible),regions=len(ids),tp=tp,fp=len(ids)-tp,fn_all=len(names)-tp,fn_visible=len(visible)-tp,recall_all=tp/max(1,len(names)),recall_visible=tp/max(1,len(visible)),precision=tp/max(1,len(ids)),iou_threshold=.5,matches=matches,limitations='Mesh rasterization omits vein overlays and robot occlusion; audit only an unobstructed stationary pile scan. Poses and RGB are asynchronous snapshots.')
 foreground=np.isin(pred,ids);truth_foreground=labels>0;intersection=int(np.count_nonzero(foreground&truth_foreground))
 result['foreground']=dict(iou=intersection/max(1,int(np.count_nonzero(foreground|truth_foreground))),precision=intersection/max(1,int(foreground.sum())),recall=intersection/max(1,int(truth_foreground.sum())),note='Union of visible leaf regions, distinct from per-leaf instance recall')
 from height_regions import split_height
 height_labels,height_regions=split_height(rgb,xyz,mask);hp=[]
 for g in visible:
  a=labels==g
  for h in height_regions:
   b=height_labels==h['id'];iou=np.count_nonzero(a&b)/max(1,np.count_nonzero(a|b))
   if iou>=.5:hp.append((iou,g,h['id']))
 hg=set();hd=set()
 for _,g,d in sorted(hp,reverse=True):
  if g not in hg and d not in hd:hg.add(g);hd.add(d)
 result['height_watershed']=dict(regions=len(height_regions),tp=len(hg),fp=len(height_regions)-len(hg),fn_all=len(names)-len(hg),fn_visible=len(visible)-len(hg),recall_all=len(hg)/max(1,len(names)),precision=len(hg)/max(1,len(height_regions)),note='Visible surface regions; no inference of fully occluded leaves')
 colors=np.random.default_rng(41).integers(40,255,(len(height_regions)+1,3),dtype=np.uint8);colors[0]=0
 cv2.imwrite(str(evidence/(prefix+'-height-regions.png')),colors[height_labels])
 annotated=rgb.copy()
 for i in ids:
  x,y,w,h,_=stats[i];color=(40,230,90) if i in matched_pred else (255,60,40);cv2.rectangle(annotated,(x,y),(x+w,y+h),color,2);cv2.putText(annotated,f'R{i}',(x,y+16),cv2.FONT_HERSHEY_SIMPLEX,.5,color,1)
 cv2.imwrite(str(evidence/(prefix+'-audit.png')),cv2.cvtColor(annotated,cv2.COLOR_RGB2BGR));np.savez_compressed(evidence/(prefix+'-instance-audit.npz'),visible_instance_labels=labels,mesh_axial_depth=depth)
 (evidence/(prefix+'-metrics.json')).write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
if __name__=='__main__':
 parser=argparse.ArgumentParser();parser.add_argument('prefix',nargs='?',default='pile-1-detect');evaluate(parser.parse_args().prefix)
