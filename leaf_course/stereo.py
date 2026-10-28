"""Depth from two RGB images only. Ideal simulated stereo calibration, no depth sensor."""
import cv2,numpy as np

class StereoDepth:
 def __init__(self,c):
  self.c=c;size=(c['width'],c['height']);K=np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
  self.baseline=c['baseline_m']
  R1,R2,P1,P2,self.Q,_,_=cv2.stereoRectify(K,np.zeros(5),K,np.zeros(5),size,np.eye(3),np.array([-self.baseline,0.,0.]),flags=cv2.CALIB_ZERO_DISPARITY,alpha=0)
  self.maps=[cv2.initUndistortRectifyMap(K,np.zeros(5),R,P,size,cv2.CV_32FC1) for R,P in [(R1,P1),(R2,P2)]]
  args=dict(numDisparities=128,blockSize=5,P1=8*25,P2=32*25,disp12MaxDiff=1,preFilterCap=31,uniquenessRatio=5,speckleWindowSize=60,speckleRange=2,mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
  self.left=cv2.StereoSGBM_create(minDisparity=0,**args);self.right=cv2.StereoSGBM_create(minDisparity=-128,**args)
 def compute(self,left,right):
  rect=[cv2.remap(im,*maps,cv2.INTER_LINEAR) for im,maps in zip([left,right],self.maps)]
  gray=[cv2.cvtColor(im,cv2.COLOR_RGB2GRAY) for im in rect]
  dl=self.left.compute(*gray).astype(np.float32)/16.;dr=self.right.compute(gray[1],gray[0]).astype(np.float32)/16.
  v,u=np.indices(dl.shape);ur=np.rint(u-dl).astype(int);inside=(ur>=0)&(ur<dl.shape[1]);corr=dr[v,np.clip(ur,0,dl.shape[1]-1)]
  valid=inside&(dl>2)&(dl<127)&(np.abs(dl+corr)<1.)
  depth=np.full(dl.shape,np.nan,np.float32);depth[valid]=self.c['fx']*self.baseline/dl[valid]
  valid&=(depth>.12)&(depth<2.5);depth[~valid]=np.nan
  return rect[0],rect[1],depth,dl,valid
