"""Visible surface watershed from measured height, independent of simulator labels."""
import cv2,numpy as np
from scipy.ndimage import gaussian_filter,maximum_filter,label

def split_height(rgb,world,mask,min_pixels=80):
 valid=(mask>0)&np.isfinite(world).all(axis=-1)&(world[:,:,2]>.0003)&(world[:,:,2]<.15)
 valid&=(world[:,:,0]>.26)&(world[:,:,0]<.75)&(world[:,:,1]>.04)&(world[:,:,1]<.46)
 weight=gaussian_filter(valid.astype(float),2.)
 heights=gaussian_filter(np.where(valid,world[:,:,2],0.),2.)/np.maximum(weight,1e-8)
 distance=cv2.distanceTransform(valid.astype('uint8'),cv2.DIST_L2,5)
 # Height maxima seed exposed folds. A small distance term favors interior points.
 relief=heights+.00015*distance
 peaks=valid&(distance>4)&(relief>=maximum_filter(np.where(valid,relief,-1),size=31))
 nlabels,n=label(peaks)
 markers=np.zeros(valid.shape,np.int32);markers[~valid]=1
 for i in range(1,n+1):markers[nlabels==i]=i+1
 if not n:return np.zeros(valid.shape,np.int32),[]
 # Measured height edges separate levels; RGB edges help stop at visible leaf edges.
 gray=cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY).astype(float)/255
 hx=cv2.Sobel(heights,cv2.CV_64F,1,0);hy=cv2.Sobel(heights,cv2.CV_64F,0,1)
 edge=np.clip(np.hypot(hx,hy)/.015+.2*gray,0,1)
 image=np.repeat(np.uint8(edge*255)[:,:,None],3,axis=2)
 cv2.watershed(image,markers)
 output=np.zeros(valid.shape,np.int32);regions=[]
 for i in range(2,n+2):
  selected=(markers==i)&valid
  if selected.sum()<min_pixels:continue
  points=world[selected];identity=len(regions)+1;output[selected]=identity
  regions.append(dict(id=identity,pixels=int(selected.sum()),median_height_mm=float(np.median(points[:,2])*1000),center=np.median(points,axis=0).tolist()))
 return output,regions
