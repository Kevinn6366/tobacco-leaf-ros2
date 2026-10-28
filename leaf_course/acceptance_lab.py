"""Additional reproducible depth-strata and moving-feed kinematic experiments.
These are synthetic experiments, not claimed as Gazebo conveyor or real-camera tests.
"""
from pathlib import Path
import json,cv2,numpy as np
from scipy.signal import find_peaks
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from perception import segment
from motion_compensation import predict_position
P=Path(__file__).resolve().parent;E=P/'evidence';rng=np.random.default_rng(20261027)
d=np.load(E/'overlap-depth-labels.npz');depth=d['depth'];truth=d['instance_labels']
rgb=cv2.cvtColor(cv2.imread(str(E/'overlap-detections.png')),cv2.COLOR_BGR2RGB)
# Use foreground depth, independent of truth labels, to identify observable strata.
height=(.60-depth)*1000;foreground=height>.7
hist,edges=np.histogram(height[foreground],bins=np.arange(0,12,.2));peaks,_=find_peaks(hist,distance=7,prominence=max(5,hist.max()*.06));centers=(edges[:-1]+edges[1:])/2;levels=centers[peaks]
labels=np.zeros(depth.shape,np.int32);regions=[]
for i,h in enumerate(levels):
 mask=(foreground&(abs(height-h)<.9)).astype('uint8')
 # Bridge narrow occlusion gaps within one height stratum (31 px ~= 17 mm).
 closed=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(31,31)))
 n,cc,stats,cents=cv2.connectedComponentsWithStats(closed)
 for k in range(1,n):
  if stats[k,4]<80:continue
  observed=(cc==k)&(mask>0)
  labels[observed]=len(regions)+1;regions.append(dict(height_mm=float(h),pixels=int(observed.sum())))
pairs=[]
for gt in range(1,25):
 for pred in range(1,len(regions)+1):
  a=truth==gt;b=labels==pred;iou=np.sum(a&b)/max(1,np.sum(a|b))
  if iou>=.5:pairs.append((iou,gt,pred))
ug=set();up=set()
for _,g,p in sorted(pairs,reverse=True):
 if g not in ug and p not in up:ug.add(g);up.add(p)
metrics=dict(tp=len(ug),fp=len(regions)-len(up),fn=24-len(ug),iou_threshold=.5,recall=len(ug)/24,precision=len(ug)/max(1,len(regions)),visible_instances=int(sum(np.any(truth==i) for i in range(1,25))),height_peaks_mm=levels.tolist(),gap_closing_kernel_pixels=31,note='Height clustering plus small-gap closing; labels include only observed depth pixels, not inferred hidden surfaces.')
fig,axs=plt.subplots(1,2,figsize=(11,4));axs[0].imshow(labels,cmap='nipy_spectral');axs[0].set_title('Depth strata + connected regions');axs[0].axis('off');axs[1].bar(centers,hist,width=.18);axs[1].set(xlabel='Visible height (mm)',ylabel='Pixels',title='Peaks inferred from measured depth');fig.tight_layout();fig.savefig(E/'height-segmentation.png',dpi=150);plt.close(fig)
(E/'height-segmentation.json').write_text(json.dumps(metrics,indent=2))
# Moving-feed plant x(t)=x0+v*t. Encoder and actuation timing are noisy independent observations.
rows=[]
for v in [0.,.03,.06,.10]:
 for delay in [.1,.25,.5]:
  n=1000;x0=rng.uniform(.3,.6,n);actual_delay=np.maximum(0,delay+rng.normal(0,.01,n));actual=x0+v*actual_delay
  observed=x0+rng.normal(0,.001,n);encoder=v+rng.normal(0,.001,n)
  points=np.column_stack([observed,np.zeros(n),np.zeros(n)]);velocities=np.column_stack([encoder,np.zeros(n),np.zeros(n)])
  prediction=predict_position(points,velocities,0.,delay)[:,0]
  no=abs(actual-observed)*1000;yes=abs(actual-prediction)*1000
  rows.append(dict(speed_m_s=v,command_latency_s=delay,trials=n,uncompensated_mean_mm=float(no.mean()),compensated_mean_mm=float(yes.mean()),compensated_p95_mm=float(np.percentile(yes,95)),uncompensated_within_5mm=float(np.mean(no<=5)),compensated_within_5mm=float(np.mean(yes<=5))))
fig,ax=plt.subplots(figsize=(9,4));x=np.arange(len(rows));ax.bar(x-.18,[r['uncompensated_mean_mm'] for r in rows],.36,label='No prediction');ax.bar(x+.18,[r['compensated_mean_mm'] for r in rows],.36,label='Encoder prediction');ax.set_xticks(x,[f'{r["speed_m_s"]}/{r["command_latency_s"]}' for r in rows],rotation=35);ax.set(xlabel='Speed (m/s) / latency (s)',ylabel='Mean alignment error (mm)',title='Separate moving-feed kinematic experiment');ax.legend();fig.tight_layout();fig.savefig(E/'motion-compensation.png',dpi=150);plt.close(fig)
(E/'motion-compensation.json').write_text(json.dumps(dict(evidence_type='synthetic kinematic moving-feed experiment; main Gazebo scene is stationary ground heap',seed=20261027,formula='x_grasp=x_detect+v_encoder*latency',pixel_localization_std_m=.001,encoder_std_m_s=.001,timing_std_s=.01,rows=rows),indent=2))
print(json.dumps(metrics));print('Moving-feed trials:',sum(r['trials'] for r in rows))
