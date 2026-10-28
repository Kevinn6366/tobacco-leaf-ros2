"""Infer an in-plane grasp direction from RGB ridges reprojected through stereo depth."""
import cv2,numpy as np

def ridge_segments(rgb,world,mask):
 edges=cv2.Canny(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),10,35)
 lines=cv2.HoughLinesP(edges,1,np.pi/180,threshold=16,minLineLength=24,maxLineGap=6)
 out=[]
 if lines is None:return out
 def point(x,y):
  points=world[max(0,y-2):y+3,max(0,x-2):x+3];valid=np.all(np.isfinite(points),axis=-1)&(mask[max(0,y-2):y+3,max(0,x-2):x+3]>0)
  return np.median(points[valid],axis=0) if valid.sum()>5 else None
 for line in lines[:,0]:
  x1,y1,x2,y2=map(int,line);mx,my=(x1+x2)//2,(y1+y2)//2
  if not mask[my,mx]:continue
  a,b=point(x1,y1),point(x2,y2);middle=point(mx,my)
  if a is None or b is None or middle is None:continue
  delta=b-a;length=np.linalg.norm(delta[:2])
  if not .020<length<.22:continue
  yaw=float(np.arctan2(delta[1],delta[0]));yaw=(yaw+np.pi/2)%np.pi-np.pi/2
  out.append(dict(center=middle,length=length,yaw=yaw,pixels=line.tolist()))
 return out

def direction_at(candidate,segments):
 g=np.array(candidate['grasp']);near=[s for s in segments if np.linalg.norm(s['center'][:2]-g[:2])<.055 and abs(s['center'][2]-g[2])<.015]
 if not near:return 0.,None
 line=max(near,key=lambda s:s['length']/(1+np.linalg.norm(s['center'][:2]-g[:2])/.025))
 candidate['grasp']=line['center'].tolist()
 candidate['height_mm']=float(line['center'][2]*1000)
 candidate['grasp_source']='RGB ridge midpoint with foreground stereo depth'
 return line['yaw'],line['pixels']
