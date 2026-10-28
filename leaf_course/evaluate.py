"""Reproducible perception/calibration tests. Synthetic evidence is labelled as such."""
from pathlib import Path
import json,cv2,numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from perception import segment,C,hand_camera
P=Path(__file__).resolve().parent;E=P/'evidence';E.mkdir(exist_ok=True)
def scene(seed,overlap=False):
 rng=np.random.default_rng(seed);H,W=960,1280;rgb=np.full((H,W,3),[65,83,92],np.uint8);depth=np.full((H,W),.60,np.float32);truth=np.zeros((H,W),np.int32)
 for i in range(24):
  if overlap:x=200+(i//3%4)*285+rng.uniform(-15,15);y=300+(i//12)*320+rng.uniform(-15,15)
  else:x=110+(i%6)*210+rng.uniform(-15,15);y=125+(i//6)*230+rng.uniform(-15,15)
  theta=rng.uniform(-np.pi,np.pi);length=rng.uniform(115,165);width=rng.uniform(37,65)
  t=np.linspace(0,1,60);half=width/2*np.sin(np.pi*t)**.85*(.65+.35*t);xx=(t-.5)*length;xy=np.vstack([np.c_[xx,half],np.c_[xx[::-1],-half[::-1]]]);R=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]);pts=np.int32(xy@R.T+[x,y]);mask=np.zeros((H,W),np.uint8);cv2.fillPoly(mask,[pts],255)
  leaf_color=[rng.integers(125,175),rng.integers(65,88),rng.integers(15,30)];height=.002+.003*(i%3) if overlap else rng.uniform(.0015,.009)
  yy,xxp=np.nonzero(mask);zz=.60-height+.0003*np.sin(xxp*.15)+rng.normal(0,.0002,len(yy));closer=zz<depth[yy,xxp];yy,xxp,zz=yy[closer],xxp[closer],zz[closer];truth[yy,xxp]=i+1;depth[yy,xxp]=zz;rgb[yy,xxp]=leaf_color
  a=tuple(np.int32([x,y]-R[:,0]*length*.4));b=tuple(np.int32([x,y]+R[:,0]*length*.4));vein=np.zeros((H,W),np.uint8);cv2.line(vein,a,b,255,1);rgb[(vein>0)&(truth==i+1)]=[89,50,14]
 noise=rng.normal(0,2,rgb.shape);rgb=np.uint8(np.clip(rgb.astype(float)+noise,0,255));return rgb,depth,truth

def metrics(rgb,truth):
 mask,n,lab,stats,_=segment(rgb);ids=[i for i in range(1,n) if stats[i,4]>=80];pairs=[]
 for gt in range(1,25):
  a=truth==gt
  for pred in ids:
   b=lab==pred;inter=np.sum(a&b);iou=inter/max(1,np.sum(a|b))
   if iou>=.5:pairs.append((iou,gt,pred))
 usedg=set();usedp=set()
 for iou,g,p in sorted(pairs,reverse=True):
  if g not in usedg and p not in usedp:usedg.add(g);usedp.add(p)
 ann=rgb.copy()
 for i in ids:
  x,y,w,h,_=stats[i];cv2.rectangle(ann,(x,y),(x+w,y+h),(30,245,130),2);cv2.putText(ann,str(i),(x,y-4),cv2.FONT_HERSHEY_SIMPLEX,.55,(30,245,130),1)
 return dict(tp=len(usedg),fp=len(ids)-len(usedp),fn=24-len(usedg),regions=len(ids),iou_threshold=.5),ann,mask
records=[]
for seed in range(10):
 rgb,depth,truth=scene(20261008+seed);m,ann,mask=metrics(rgb,truth);m['seed']=20261008+seed;records.append(m)
 if seed==0:
  cv2.imwrite(str(E/'dataset-24-rgb.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR));cv2.imwrite(str(E/'dataset-24-detections.png'),cv2.cvtColor(ann,cv2.COLOR_RGB2BGR));np.savez_compressed(E/'dataset-24-depth-labels.npz',depth=depth,instance_labels=truth)
  v,u=np.nonzero((mask>0));z=depth[v,u];pts=np.c_[(u-639.5)*z/1108.5,(v-479.5)*z/1108.5,.6-z];sel=np.arange(0,len(pts),4);fig=plt.figure(figsize=(9,6));ax=fig.add_subplot(111,projection='3d');ax.scatter(*pts[sel].T,c=pts[sel,2]*1000,s=1,cmap='viridis');ax.set(xlabel='X / m',ylabel='Y / m',zlabel='Height / m',title='24 randomized leaves: pinhole depth reconstruction');fig.tight_layout();fig.savefig(E/'dataset-pointcloud.png',dpi=160);plt.close(fig)
rgb,depth,truth=scene(20261008,True);overlap,ann,mask=metrics(rgb,truth);cv2.imwrite(str(E/'overlap-detections.png'),cv2.cvtColor(ann,cv2.COLOR_RGB2BGR));np.savez_compressed(E/'overlap-depth-labels.npz',depth=depth,instance_labels=truth)
# Height classes are observable surface strata, not a count of hidden leaves.
heights=(.6-depth)[mask>0]*1000;hist,bins=np.histogram(heights,bins=np.arange(0,12,.25));fig,ax=plt.subplots(figsize=(7,3));ax.bar((bins[:-1]+bins[1:])/2,hist,width=.24);ax.set(xlabel='Visible height above belt / mm',ylabel='Pixels',title='Overlapping scene: depth strata at 2, 5 and 8 mm');fig.tight_layout();fig.savefig(E/'height-strata.png',dpi=160);plt.close(fig)
TP=sum(x['tp'] for x in records);FP=sum(x['fp'] for x in records);FN=sum(x['fn'] for x in records)
# Independent analytic perturbation study: fixed 3D targets back-projected using perturbed calibration.
rng=np.random.default_rng(42);uv=rng.uniform([60,50],[580,430],(1000,2));z=rng.uniform(.35,.7,1000);cam=np.c_[z,-(uv[:,0]-C['cx'])*z/C['fx'],-(uv[:,1]-C['cy'])*z/C['fy']];cal=[]
for name,trans,angle,fscale in [('nominal',0,0,1),('translation_2mm',.002,0,1),('rotation_0.5deg',0,.5,1),('focal_1percent',0,0,1.01),('combined',.002,.5,1.01)]:
 p=cam.copy();p[:,1:]/=fscale;p=p@Rotation.from_euler('z',angle,degrees=True).as_matrix().T+[trans,0,0];err=np.linalg.norm(p-cam,axis=1)*1000;cal.append(dict(case=name,mean_mm=float(np.mean(err)),p95_mm=float(np.percentile(err,95)),max_mm=float(np.max(err)),within_5mm=float(np.mean(err<5))))
fig,ax=plt.subplots(figsize=(8,4));ax.bar([c['case'] for c in cal],[c['p95_mm'] for c in cal],color='#237c83');ax.axhline(5,color='#c45444',ls='--',label='5 mm design target');ax.set(ylabel='95th percentile error / mm',title='Calibration sensitivity (1000 synthetic rays)');ax.tick_params(axis='x',rotation=15);ax.legend();fig.tight_layout();fig.savefig(E/'calibration-error.png',dpi=160);plt.close(fig)
result=dict(evidence_type='synthetic randomized raster/depth benchmark, not real tobacco data',seeds=records,precision=TP/max(1,TP+FP),recall=TP/max(1,TP+FN),tp=TP,fp=FP,fn=FN,overlap_instance_metrics=overlap,overlap_note='Connected components merge overlapping instances; report these false negatives rather than claiming hidden leaf detection.',calibration=cal)
(E/'perception-calibration.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
