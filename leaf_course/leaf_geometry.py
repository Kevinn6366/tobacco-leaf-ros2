"""One convex, watertight prism per elastic leaf segment, shared by rendering/physics."""
import numpy as np

THICKNESS=.0015
# A coarse lanceolate outline; deformation belongs to joints, not fake visual waves.
STATIONS=np.array([[-.105,.004],[-.035,.075],[.035,.082],[.105,.004]])

def patch(section,row):
 x0,w0=STATIONS[section];x1,w1=STATIONS[section+1]
 cx=(x0+x1)/2;cy=row*(w0+w1)/6
 low,high=(row-.5)/3,(row+.5)/3
 polygon=np.array([[x0,low*w0],[x1,low*w1],[x1,high*w1],[x0,high*w0]])-[cx,cy]
 return np.array([cx,cy,0.]),polygon

def properties(poly,mass,thickness=THICKNESS):
 x,y=poly.T;u,v=np.roll(poly,-1,axis=0).T;c=x*v-u*y;area=c.sum()/2
 assert area>0
 center=np.array([((x+u)*c).sum(),((y+v)*c).sum()])/(6*area)
 xx=((x*x+x*u+u*u)*c).sum()/(12*area)-center[0]**2
 yy=((y*y+y*v+v*v)*c).sum()/(12*area)-center[1]**2
 xy=((2*x*y+x*v+u*y+2*u*v)*c).sum()/(24*area)-center.prod()
 inertia=dict(ixx=mass*(yy+thickness**2/12),iyy=mass*(xx+thickness**2/12),izz=mass*(xx+yy),ixy=-mass*xy,ixz=0.,iyz=0.)
 return float(area),[float(center[0]),float(center[1]),0.],{k:float(v) for k,v in inertia.items()}

def prism(poly,thickness=THICKNESS):
 v=np.vstack([np.c_[poly,np.full(4,-thickness/2)],np.c_[poly,np.full(4,thickness/2)]])
 f=[(0,2,1),(0,3,2),(4,5,6),(4,6,7)]
 for i in range(4):
  j=(i+1)%4;f.extend([(i,j,j+4),(i,j+4,i+4)])
 return v,np.array(f)

def write_obj(path,vertices,faces):
 path.write_text('\n'.join(['v %.10g %.10g %.10g'%tuple(v) for v in vertices]+['f %d %d %d'%tuple(i+1 for i in f) for f in faces])+'\n')

def surface_veins(poly,row):
 # Surface-only color marks, 10 micrometres above the physical top face.
 vertices=[];faces=[];x0,x1=poly[:,0].min(),poly[:,0].max()
 def bounds(x):
  t=(x-x0)/(x1-x0);return (1-t)*poly[0,1]+t*poly[1,1],(1-t)*poly[3,1]+t*poly[2,1]
 for x in np.linspace(x0+.012,x1-.012,3):
  a,b=bounds(x);half=min(.0004,(x1-x0)/20);k=len(vertices)
  vertices.extend([[x-half,a+.001,THICKNESS/2+.00001],[x+half,a+.001,THICKNESS/2+.00001],[x+half,b-.001,THICKNESS/2+.00001],[x-half,b-.001,THICKNESS/2+.00001]])
  faces.extend([(k,k+1,k+2),(k,k+2,k+3)])
 if row==0:
  k=len(vertices);vertices.extend([[x0,-.0004,THICKNESS/2+.00001],[x1,-.0004,THICKNESS/2+.00001],[x1,.0004,THICKNESS/2+.00001],[x0,.0004,THICKNESS/2+.00001]]);faces.extend([(k,k+1,k+2),(k,k+2,k+3)])
 return np.array(vertices),np.array(faces)
