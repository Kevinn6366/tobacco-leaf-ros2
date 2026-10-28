"""Three-millimetre lead-in bevel, with exact composite mass properties."""
import numpy as np
from leaf_geometry import properties

def finger(side):
 sign=1 if side=='left' else -1
 poly=np.array([[-.005,.043],[-.005,.050],[-.008,.050]])
 poly[:,0]*=sign
 if sign<0:poly=poly[::-1]
 vertices=np.vstack([np.c_[np.full(3,-.014),poly],np.c_[np.full(3,.014),poly]])
 faces=[(0,2,1),(3,4,5)]
 for i in range(3):
  j=(i+1)%3;faces.extend([(i,j,j+3),(i,j+3,i+3)])
 area=properties(poly,1.)[0];vb=.028*.010*.050;vw=area*.028;mb=.15*vb/(vb+vw);mw=.15-mb
 _,uv,mom=properties(poly,mw,.028);cb=np.array([0,0,.025]);cw=np.array([0,uv[0],uv[1]]);center=(mb*cb+mw*cw)/.15
 ib=mb/12*np.diag([.010**2+.050**2,.028**2+.050**2,.028**2+.010**2])
 iw=np.array([[mom['izz'],0,0],[0,mom['ixx'],mom['ixy']],[0,mom['ixy'],mom['iyy']]])
 inertia=ib+iw
 for mass,c in [(mb,cb),(mw,cw)]:
  delta=c-center;inertia+=mass*(np.dot(delta,delta)*np.eye(3)-np.outer(delta,delta))
 return vertices,np.array(faces),center,inertia
